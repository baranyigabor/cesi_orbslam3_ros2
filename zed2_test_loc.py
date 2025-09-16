import pyzed.sl as sl
import cv2
import numpy as np

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Define configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Set the depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units

    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(err)
        exit(1)

    # Define a runtime parameters object
    runtime_parameters = sl.RuntimeParameters()

    # Create objects to store images and depth
    image = sl.Mat()
    depth = sl.Mat()

    camera_info = zed.get_camera_information()

    # Accessing the camera resolution
    resolution = camera_info.camera_resolution
    width = resolution.width
    height = resolution.height

    # Accessing the camera focal length
    focal_length = camera_info.calibration_parameters.left_cam.fx

    # Accessing the camera model
    camera_model = camera_info.camera_model

    # Print some of the camera parameters
    print(f"Resolution: {width} x {height}")
    print(f"Focal Length: {focal_length}")
    print(f"Camera Model: {camera_model}")

    calibration_params = camera_info.calibration_parameters
    left_cam_params = calibration_params.left_cam

    # Intrinsic parameters of the left camera
    fx = left_cam_params.fx  # Focal length in x
    fy = left_cam_params.fy  # Focal length in y
    cx = left_cam_params.cx  # Principal point in x
    cy = left_cam_params.cy  # Principal point in y

    # Distortion coefficients of the left camera
    dist_coeffs = left_cam_params.disto

    # Print some of the intrinsic parameters
    print(f"Intrinsic Parameters: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    print(f"Distortion Coefficients: {dist_coeffs}")

    zed.close()
    print('done---exit')
    exit(0)

    # Capture 50 frames and retrieve images
    for i in range(50):
        # Grab an image frame
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Retrieve the depth map
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

            # Convert the images to OpenCV format
            image_ocv = image.get_data()
            depth_ocv = depth.get_data()
            print(image_ocv.shape)
            print(depth_ocv.shape)

            # Save the RGB image
            cv2.imwrite(f"rgb_image_{i}.png", image_ocv)

            # Save the depth image - may need conversion depending on your use case
            cv2.imwrite(f"depth_image_{i}.png", depth_ocv)

            # Display the image and depth (optional)
            #cv2.imshow("Image", image_ocv)
            #cv2.imshow("Depth", depth_ocv)
            # cv2.waitKey(1)
        else:
            print('alma')

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
