import http.client
import requests
import numpy as np
import cv2
import msgpack
import base64

iphoneIp = "http://192.168.0.141/"

# The maximum line length for HTTP headers
# This is necessary to avoid the "line too long" error when receiving large headers, and in our case the depth data is 512964 bytes long
http.client._MAXLINE = 600000

while (True):
    # Send a get request to the http server running on the iphone
    response = requests.get(iphoneIp)

    # Load image from the request body
    arr = np.asarray(bytearray(response.content), dtype=np.uint8)
    image = cv2.imdecode(arr, -1)

    # Split image into color channels
    b, g, r, d = cv2.split(image)  

    # Display the received image 
    cv2.imshow('Colored image', image)  

    # Extract the X-Depth-Message-Pack header
    depth_data_base64 = response.headers.get("X-Depth-Message-Pack")
    #print(f"Size of depth_data_base64 in bytes: {len(depth_data_base64.encode('utf-8'))}")
    if depth_data_base64:
        # Decode the base64-encoded data
        depth_data_bytes = base64.b64decode(depth_data_base64)
        
        # Decode the MessagePack data
        depth_data = msgpack.unpackb(depth_data_bytes, raw=False)
        
        # Create a numpy array from the depth data
        depth_arr = np.asarray(depth_data)

        max_depth = 10
        min_depth = 0
        # Normalize depth_arr to scale values between 0 and 255 using specific min and max (0 and 10)
        depth_normalized = np.clip(depth_arr / max_depth * 255, min_depth, 255)
        
        # Convert to uint8 for display
        depth_display = depth_normalized.astype(np.uint8)
        
        # Display depth as grayscale
        cv2.imshow('Depth', depth_display)

    cv2.waitKey(200) # Specifying a too low number can cause performance issues