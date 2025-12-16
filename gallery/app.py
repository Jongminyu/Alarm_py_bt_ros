import os
from flask import Flask, render_template, jsonify, send_from_directory
from datetime import datetime

app = Flask(__name__)

# Base paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)
CAPTURES_DIR = os.path.join(PROJECT_ROOT, 'captures')

# Ensure captures dir exists
if not os.path.exists(CAPTURES_DIR):
    os.makedirs(CAPTURES_DIR)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/photos')
def get_photos():
    photos = []
    # format: capture_YYYYMMDD_HHMMSS.jpg
    # or just any image, we try to parse date
    if os.path.exists(CAPTURES_DIR):
        for f in os.listdir(CAPTURES_DIR):
            if f.lower().endswith(('.png', '.jpg', '.jpeg')):
                # Try to parse date from filename if possible
                # Expected: capture_20251216_103000.jpg
                date_str = None
                try:
                    parts = f.split('_')
                    if len(parts) >= 2:
                        date_part = parts[1] # 20251216
                        # Validation
                        if len(date_part) == 8 and date_part.isdigit():
                            date_str = f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:]}" # YYYY-MM-DD
                except:
                    pass
                
                # Fallback: Use file creation time if name parse fails
                if not date_str:
                    full_path = os.path.join(CAPTURES_DIR, f)
                    ctime = os.path.getctime(full_path)
                    dt = datetime.fromtimestamp(ctime)
                    date_str = dt.strftime('%Y-%m-%d')

                photos.append({
                    'filename': f,
                    'date': date_str
                })
    
    return jsonify(photos)

@app.route('/captures/<path:filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURES_DIR, filename)

if __name__ == '__main__':
    print(f"📂 Serving photos from: {CAPTURES_DIR}")
    print("🌍 Calendar running at http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=True)
