import asyncio
import argparse
import cProfile

from modules.utils import set_config

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='config.yaml', help='Path to the configuration file (default: --config=config.yaml)')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
from modules.bt_runner import BTRunner
bt_runner = BTRunner(config)


async def loop():
    from modules.base_bt_nodes import Status
    while bt_runner.running:
        bt_runner.handle_keyboard_events()
        
        # Render FIRST so we see the tree state even if step() blocks (e.g. Time UI)
        bt_runner.render()
        
        if not bt_runner.paused:
            await bt_runner.step()
            
            # Check if the root node has succeeded (Ending played)
            if bt_runner.agent.tree.status == Status.SUCCESS:
                print("Mission Cycle Complete. Restarting Alarm Mode...")
                # No break, let it loop. 
                pass
                
        bt_runner.clock.tick(bt_runner.bt_tick_rate)

    bt_runner.close()



if __name__ == "__main__":
    import subprocess
    import sys
    import os

    # Launch the Stop Button UI in a separate process
    ui_process = subprocess.Popen([sys.executable, "modules/stop_button_ui.py"])
    
    # Launch the Camera Service in a separate process
    camera_process = subprocess.Popen([sys.executable, "modules/camera_service.py"])
    
    # YOLO Removed (User runs manually)

    try:
        if config['bt_runner']['profiling_mode']:
            cProfile.run('main()', sort='cumulative')
        else:
            asyncio.run(loop())
    finally:
        # Ensure the processes are killed when main exits
        ui_process.terminate()
        camera_process.terminate()