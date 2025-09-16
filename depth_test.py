import cv2
import numpy as np
import pyzed.sl as sl

def main():
    # Initialize the ZED camera
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER       # Set the depth coordinate units to millimeters

    # Open the camera
    if not zed.is_opened():
        print("Opening ZED Camera...")
        status = zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            exit(-1)

    # Create a Mat to store images
    image = sl.Mat()
    depth = sl.Mat()

    # Capture and display loop
    key = ''
    while key != ord('q'):  # Press 'q' to exit
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image
            zed.retrieve_image(image, sl.VIEW.LEFT)

            # Retrieve the depth map
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

            # Convert the images to OpenCV format
            image_ocv = image.get_data()
            depth_ocv = depth.get_data()

            # Handle invalid values in the depth map
            depth_ocv[np.isnan(depth_ocv)] = 0
            depth_ocv[np.isinf(depth_ocv)] = 0

            # Normalize the depth for visualization
            depth_normalized = cv2.normalize(depth_ocv, None, 0, 255, cv2.NORM_MINMAX)
            depth_display = np.uint8(depth_normalized)

            # Display the depth map and raw image
            cv2.imshow("Depth Map", depth_display)
            cv2.imshow("Raw Image", image_ocv)

            key = cv2.waitKey(10)

    # Close the camera and window
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
