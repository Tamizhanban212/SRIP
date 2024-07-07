import cv2

# Function to detect circular disks and display transformed coordinates
def detect_and_display():
    # Open webcam
    cap = cv2.VideoCapture(0)
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret:
            # Convert frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply Hough Circle Transform to detect circles
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=1000,
                                       param1=120, param2=40, minRadius=10, maxRadius=300)
            
            if circles is not None:
                circles = circles[0, :].round().astype("int")
                
                for (x, y, r) in circles:
                    # Draw the circle detected
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    
                    # Draw the center of the circle
                    cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
                    
                    # Apply coordinate transformation
                    x_new = y * 0.00039 + 0.052
                    y_new = (x - 320) * 0.00039
                    r_new = r * 0.039 * 2  # Convert radius to meters
                    
                    # Display transformed coordinates and radius
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    text_transformed = f'Transformed Center: ({x_new:.2f}, {y_new:.2f})'
                    text_radius = f'Diameter: {r_new:.2f} cms'
                    
                    # Get text size
                    text_transformed_size = cv2.getTextSize(text_transformed, font, 0.5, 2)[0]
                    text_radius_size = cv2.getTextSize(text_radius, font, 0.5, 2)[0]
                    
                    # Calculate text positions (ensuring text is within the frame)
                    text_transformed_pos = (max(10, x - text_transformed_size[0] // 2),
                                            max(30, y - r - 30))
                    text_radius_pos = (max(10, x - text_radius_size[0] // 2),
                                       max(20, y - r - 10))
                    
                    # Draw text on frame
                    cv2.putText(frame, text_transformed, text_transformed_pos,
                                font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(frame, text_radius, text_radius_pos,
                                font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            
            # Display the frame
            cv2.imshow('Webcam', frame)
            
        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the capture and destroy any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Run the function
detect_and_display()



# 100 pixels -----> 3.9 cm
# 1 pixel -------> 0.039 cm
# 1 cm ----------> 25.64 pixels
# 18 cm x 25.3 cm (camera frame)
# 36.5 cm x 16.2 cm (L framework)