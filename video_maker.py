import cv2
import os
import glob
import natsort

# === CONFIGURATION ===
input_folder = "/media/gables/Data/Data/Drone_Mamba/iphone/nas/2025_05_28_taskname_Jr84Zu_frames/"  # <-- change this
output_video = "output_video_white_pillar_new.mp4"
fps = 12

# === Find and sort image files ===
image_files = glob.glob(os.path.join(input_folder, "colorImg_*.jpg"))
image_files = natsort.natsorted(image_files)

# === Sanity check ===
if not image_files:
    raise FileNotFoundError("No matching 'colorImg_*.jpg' files found in the specified folder.")

# === Read the first image to get frame size ===
first_frame = cv2.imread(image_files[0])
height, width, _ = first_frame.shape

# === Define the video writer ===
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4
video_writer = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

# === Write frames ===
for img_path in image_files:
    frame = cv2.imread(img_path)
    if frame is not None:
        video_writer.write(frame)
    else:
        print(f"Warning: Could not read image {img_path}")

video_writer.release()
print(f"Video saved to {output_video}")
