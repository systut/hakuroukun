#!/usr/bin/env python3
import cv2
image = cv2.imread("saved_map.png", cv2.IMREAD_GRAYSCALE)
cv2.imwrite("edited_map.pgm", image, [cv2.IMWRITE_PXM_BINARY, 1])
