import cv2
import numpy as np
import os
import random
import time

# Resize for better visualization
def resize_image(image, max_width=1000, max_height=800):
    h, w = image.shape[:2]
    scale = min(max_width / w, max_height / h)
    if scale < 1:
        image = cv2.resize(image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return image

def get_hsv_at_pixel(hsv_image, x, y):
    # Get the HSV value at the specified (x, y) coordinate
    h, s, v = hsv_image[y, x]  # OpenCV uses (row, column) format

    return (int(h), int(s), int(v))

def detect_orange_hammer(image):
    height, width = image.shape[:2]
    total_area = float(width * height)
    print(f"w, h: {width}, {height}")

    # Apply bilateral filter to reduce noise while keeping edges sharp
    blurred = image
    blurred = cv2.bilateralFilter(image, 9, 85, 85)
    #blurred = cv2.GaussianBlur(blurred, (21, 21), 0)

    # Convert to HSV color space
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    print(f"HSV: {get_hsv_at_pixel(hsv, 284, 282)}")

    # Extract Hue, Saturation, and Value channels
    hue, sat, val = cv2.split(hsv)
    #x = 282
    #y = 284
    # pixel_hue = hue[y, x]
    # pixel_sat = sat[y, x]
    # pixel_val = val[y, x]
    #print(f"Hue = {pixel_hue}, Sat = {pixel_sat}, Val = {pixel_val}")

    # Create a base mask of all False values
    mask = np.zeros_like(hue, dtype=np.uint8)

    # Define threshold for "Saturation + Value must be above a certain level"
    min_sum_sv = 20  # Adjust as needed

    # Mask for red-orange range (Hue: 1-10)
    #mask_red_orange = (hue >= 5) & (hue <= 10) & (sat >= 100) & (val >= 120) & ((sat + val) >= min_sum_sv)

    # Mask for yellow-orange range (Hue: 10-17)
    mask_yellow_orange = (hue > 1) & (hue <= 20) & (sat >= 120) & (val >= 120) & ((sat + val) >= min_sum_sv)

    # Combine both masks
    # mask = (mask_red_orange | mask_yellow_orange).astype(np.uint8) * 255
    mask = (mask_yellow_orange).astype(np.uint8) * 255

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("No orange hammer detected.")
        return None, None, None, blurred
    
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    largest_area = cv2.contourArea(largest_contour)
    size_percent = largest_area / total_area

    # if size_percent < 0.00005:
    #     print(f"No orange hammer detected. Largest size was {size_percent:.6f}")
    #     return None, None, None, blurred

    # Get bounding box for the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)
    x_pos = x / width  # Normalize x position
    y_pos = y / height  # Normalize y position


    # Draw the largest contour and bounding box
    output_image = blurred.copy()
    cv2.drawContours(output_image, [largest_contour], -1, (0, 255, 0), 2)
    cv2.rectangle(output_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    for contour in contours:
        contour_size = cv2.contourArea(contour)
        if contour_size / total_area < 0.00004:
            continue
        # cv2.drawContours(output_image, [contour], -1, (0, 255, 0), 1)
        # x, y, w, h = cv2.boundingRect(contour)
        # cv2.rectangle(output_image, (x, y), (x + w, y + h), (255, 0, 0), 1)

    print(f"Hammer detected at (x={x_pos:.3f}, y={y_pos:.3f}). Size as a percent of image space: {size_percent:.5f}")

    return x_pos, y_pos, size_percent, output_image

def get_average_hsv(image_path):
    """
    Computes the average HSV values of an image.

    :param image_path: Path to the input image
    :return: Tuple of (average_H, average_S, average_V)
    """
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError("Error: Could not read the image.")

    # Convert to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Compute the mean values for H, S, and V channels
    avg_h = np.mean(hsv_image[:, :, 0])
    avg_s = np.mean(hsv_image[:, :, 1])
    avg_v = np.mean(hsv_image[:, :, 2])

    return avg_h, avg_s, avg_v

def get_random_image(folder_path, i):
    """
    Selects a random image file from the given folder.
    
    :param folder_path: Path to the folder containing images.
    :return: Path to a randomly selected image or None if no images are found.
    """
    if not os.path.isdir(folder_path):
        print("Error: The provided path is not a valid directory.")
        return None

    # Get a list of image files (common formats)
    valid_extensions = (".jpg", ".jpeg", ".png", ".bmp", ".gif", ".tiff")
    image_files = [f for f in os.listdir(folder_path) if f.lower().endswith(valid_extensions)]

    if not image_files:
        print("Error: No image files found in the directory.")
        return None

    # Select a random image
    random_image = random.choice(image_files)
    #random_image = image_files[i]
    result = os.path.join(folder_path, random_image)
    print(random_image)
    return result

def color_cluster_segmentation(image, k=16):
    # Reshape image into a 2D array of pixels
    Z = image.reshape((-1,3))
    Z = np.float32(Z)

    # Apply k-means clustering
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    _, labels, centers = cv2.kmeans(Z, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # Convert centers back to uint8 and reshape
    centers = np.uint8(centers)
    segmented_image = centers[labels.flatten()].reshape(image.shape)

    # Convert to grayscale so you can do connected-components or contour detection
    gray_seg = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_seg, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Get distinct regions based on color clusters + connectivity
    num_labels, labels_im = cv2.connectedComponents(thresh)

    return segmented_image

def run():
    cap = cv2.VideoCapture(0)
    #time.sleep(1)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    # Capture a frame
    ret, frame = cap.read()
    if ret == False:
        print("Error: Could not capture image.")
        return

    x, y, size, img = detect_orange_hammer(frame)
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Check if frame was captured successfully
    # if ret:
    #     cv2.imshow("Captured Image", frame)  # Display the image in a window
    #     #cv2.imwrite("captured_image.jpg", frame)  # Save the image
    #     print("Image captured successfully.")

    #     # Wait for a key press and close the window
    #     cv2.waitKey(0)  # Press any key to close the window
    #     cv2.destroyAllWindows()
    # else:
    #     print("Error: Could not capture image.")


    # Release the camera
    cap.release()
    cv2.destroyAllWindows()

run()