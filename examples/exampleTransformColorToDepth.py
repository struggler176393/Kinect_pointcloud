import sys
import cv2

sys.path.insert(1, '../')
import pykinect_azure as pykinect

if __name__ == "__main__":

	# Initialize the library, if the library is not found, add the library path as argument
	pykinect.initialize_libraries()

	# Modify camera configuration
	device_config = pykinect.default_configuration
	device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
	device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
	# print(device_config)

	# Start device
	device = pykinect.start_device(config=device_config)

	cv2.namedWindow('Transformed Color Image',cv2.WINDOW_NORMAL)
	while True:
		
		# Get capture
		capture = device.update()

		# Get the color image from the capture
		ret, color_image = capture.get_transformed_color_image()

		if not ret:
			continue

		# Get the colored depth
		ret, depth_image = capture.get_colored_depth_image()

		# Combine both images
		combined_image = cv2.addWeighted(color_image[:,:,:3], 0.7, depth_image, 0.3, 0)
	
		# Overlay body segmentation on depth image
		cv2.imshow('Transformed Color Image',combined_image)
		
		# Press q key to stop
		if cv2.waitKey(1) == ord('q'): 
			break