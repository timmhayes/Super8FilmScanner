# from picamera2 import Picamera2, Preview
# import time

# picam2 = Picamera2() # Create a Picamera2 object.

# # Configure the camera for preview and still capture.
# # This creates a configuration with a 1920x1080 main resolution and a 640x480 low-resolution preview.
# camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, lores={"size": (640, 480)}, display="lores")
# picam2.configure(camera_config) # Apply the configuration.

# picam2.start_preview(Preview.QTGL) # Start the preview window.
# picam2.start() # Start the camera.

# time.sleep(2) # Wait for 2 seconds (allows AGC and AWB to settle).

# picam2.capture_file("test_photo.jpg") # Capture an image and save it as "test_photo.jpg".

# picam2.close() # Close the camera connection (Important for resource management).
#!/usr/bin/python3

# Normally the QtGlPreview implementation is recommended as it benefits
# from GPU hardware acceleration.

import time

from picamera2 import Picamera2, Preview

picam2 = Picamera2()
picam2.start_preview(Preview.QTGL)

preview_config = picam2.create_preview_configuration()
print(preview_config)
picam2.configure(preview_config)

picam2.start()
metadata = picam2.capture_metadata()
print(metadata)
time.sleep(30)