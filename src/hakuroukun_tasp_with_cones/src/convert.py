#!/usr/bin/env python3
import cv2
image = cv2.imread("saved_map_jikken.png", cv2.IMREAD_GRAYSCALE)
cv2.imwrite("edited_map_jikken.pgm", image, [cv2.IMWRITE_PXM_BINARY, 1])
