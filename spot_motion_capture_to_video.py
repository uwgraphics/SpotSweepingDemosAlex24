from typing import Generator
from itertools import permutations
import numpy as np
import cv2 as cv
import math
from PIL import Image, ImageDraw, ImageFont

# ARGS
CAPTURE_NAME = "7-9-2024"
DIR = f'./motion_capture/{CAPTURE_NAME}'

IGNORE = [50 , 50 , 50 ] # Ignore color
HIGHLI = "blue" # Highlight color

def main():
    """
    The main function.
    """

    # --- Make Generator to Lazy-Load Video (otherwise we consume a lot of memory) ---

    def frames(mp4_path: str) -> Generator[np.ndarray, None, None]:
        """
        Returns the next frame of the video.
        """
        cap = cv.VideoCapture(mp4_path)
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                break
            yield frame
        cap.release()

    # --- Write Results To Video  ---

    print("Writing", "Frames...")

    white_thresh = 200

    first_frame = np.array(next(frames(f'{DIR}/video.mp4')))

    # create the array that we will draw on to accumulate the scanned area
    accume_arr = np.zeros(first_frame.shape, dtype=np.uint8)
    accume = Image.fromarray(accume_arr)
    canvas = ImageDraw.Draw(accume)

    first_frame = np.array(cv.cvtColor(next(frames(f'{DIR}/video.mp4')), cv.COLOR_BGR2GRAY))

    out = cv.VideoWriter(f'{DIR}/output.mp4', cv.VideoWriter_fourcc(*'mp4v'), 30.0, (first_frame.shape[1], first_frame.shape[0]))

    start_gray = first_frame >= white_thresh
    for frame in frames(f'{DIR}/video.mp4'):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        white_mask = gray >= white_thresh

        for y, x in zip(*np.where(white_mask > 0)):
            if y <= 500 or 900 <= y:
                white_mask[y, x] = 0
                frame[y, x]      = IGNORE

        is_diff = white_mask & ~(white_mask & start_gray)

        frame[start_gray] = IGNORE

        points = []

        # find contours in the binary image
        contours, _ = cv.findContours(np.array(is_diff, dtype=np.uint8), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for c in contours:
            # calculate moments for each contour
            m = cv.moments(c)
            
            # calculate x,y coordinate of center
            cx = int(m["m10"] / (m["m00"] if m["m00"] != 0 else 1))
            cy = int(m["m01"] / (m["m00"] if m["m00"] != 0 else 1))

            if cx == 0 and cy == 0:
                continue

            points.append((cx, cy))

        frame = np.array(frame)

        # draw the box that we scanned over
        if len(points) >= 3:
            for perm in permutations(points, 3):
                canvas.polygon(perm, fill=HIGHLI)
            accume_arr = np.array(accume)
        np.putmask(frame, accume_arr != 0, accume_arr)

        out.write(frame)

    print("Done")

if __name__ == "__main__":
    main()
