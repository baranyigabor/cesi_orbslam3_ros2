import pyzed.sl as sl
import cv2

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Define configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Set the depth mode
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Define a runtime parameters object
    runtime_parameters = sl.RuntimeParameters()

    # Create objects to store images and depth
    image = sl.Mat()
    depth = sl.Mat()

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

            # Save the RGB image
            cv2.imwrite(f"rgb_image_{i}.png", image_ocv)

            # Save the depth image - may need conversion depending on your use case
            cv2.imwrite(f"depth_image_{i}.png", depth_ocv.astype(np.uint16))

            # Display the image and depth (optional)
            # cv2.imshow("Image", image_ocv)
            # cv2.imshow("Depth", depth_ocv)
            # cv2.waitKey(1)

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
