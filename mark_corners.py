import cv2

def mark_and_display_corners(webcam_index=2):
    # Open the specified webcam
    cap = cv2.VideoCapture(webcam_index)

    # Get the camera's full resolution
    if cap.isOpened():
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f'Camera resolution: {width}x{height}')
    else:
        print(f"Unable to access webcam with index {webcam_index}")
        return
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if ret:
            # Define corner coordinates
            corners = [(0, 0), (width-1, 0), (0, height-1), (width-1, height-1)]
            
            # Draw circles and text at each corner
            for (x, y) in corners:
                # Draw the corner as a filled circle
                cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)
                
                # Put text with the coordinates slightly inside the frame
                text = f'({x}, {y})'
                
                # Adjust text position based on the corner
                if x == 0 and y == 0:  # Top-left corner
                    text_position = (x + 10, y + 20)
                elif x == 0 and y == height - 1:  # Bottom-left corner
                    text_position = (x + 10, y - 10)
                elif x == width - 1 and y == 0:  # Top-right corner
                    text_position = (x - 100, y + 20)  # Increase left offset
                elif x == width - 1 and y == height - 1:  # Bottom-right corner
                    text_position = (x - 100, y - 10)  # Increase left offset
                
                # Draw the text on the frame
                cv2.putText(frame, text, text_position,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            
            # Display the frame
            cv2.imshow('Webcam', frame)
            
        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the capture and destroy any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Run the function
mark_and_display_corners()
