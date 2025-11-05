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
    while bt_runner.running:
        bt_runner.handle_keyboard_events()
        if not bt_runner.paused:
            await bt_runner.step()

    bt_runner.close()



if __name__ == "__main__":
    if config['bt_runner']['profiling_mode']:
        cProfile.run('main()', sort='cumulative')
    else:
        asyncio.run(loop())