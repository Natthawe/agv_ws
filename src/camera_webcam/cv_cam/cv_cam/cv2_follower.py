import cv2
import numpy as np

# (480, 870, 3)

no_points_count = 0
isProcessing = True
count = 0
prev_line_detection = False
last_count = 0
point_count = 0
last_point_count = 0
LeftLine = 1

while isProcessing:
    cap = cv2.VideoCapture('video.webm')
    size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    
    (w, h) = (450, 870) # Resolution
    
    while isProcessing:
        check, frame = cap.read()
        frame_rgb = frame[10:450, 0:870]
        
        frame = frame_rgb.copy()
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # rgb to gray
        # frame = cv2.medianBlur(frame, 5)
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(frame,(kernel_size, kernel_size),0)
        ret,thresh = cv2.threshold(blur_gray, 80, 255, cv2.THRESH_BINARY)
        
        diff = []
        points = []
        start_height = []
        
        left_points = []
        middle_points = []
        right_points = []

        for i in range(5):
            start_height.append(thresh.shape[0] - 1 - (45 * i))
            signed_thresh = thresh[start_height[i]].astype(np.int16) # select only one row
            # select 2 rows
            diff.append(np.diff(signed_thresh)) # derivative
        
            
            points.append(np.where(np.logical_or(diff[i] > 200, diff[i] < -200))) #maximums and minimums of derivative
            # print(f'points: {points}')
            cv2.line(frame_rgb, (0, start_height[i]), (thresh.shape[1], start_height[i]), (255, 0, 0), 2)
            
            point_count = 0
            # multiple points line drawing
            for j in range(len(points[i][0])):
                middle = points[i][0][j]
                cv2.circle(frame_rgb, (middle, start_height[i]), 5, (0, 0, 255), 2)
                cv2.circle(frame_rgb, (thresh.shape[1] // 2, start_height[i]), 5, (255, 0, 0), 2)
                cv2.line(frame_rgb, (middle, start_height[i]), (thresh.shape[1] // 2, start_height[i]), (0, 255, 0), 2)
                # cv2.putText(frame_rgb, f'{middle - thresh.shape[1] // 2}', (middle - 10, start_height[i] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                # print((middle, start_height[i]))
                # print(j)
                point_count += 1

        if point_count != last_point_count:
            last_point_count = point_count
            print(point_count)


            if point_count == 4 :
                count += 1
            
            if point_count == 2 :
                
                if count >= 2:
                    if LeftLine == 1:
                        LeftLine = 0
                    else:
                        LeftLine = 1

                    count = 0

        if LeftLine == 1:
            cv2.putText(frame_rgb, f'Left:', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        else:
            cv2.putText(frame_rgb, f'Right:', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

                

        # Display count value
        cv2.putText(frame_rgb, f'Count: {count}', (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # cv2.putText(frame_rgb, f'Left:', (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


        if LeftLine == 1:
            if point_count == 2:
                if len(points) > 0 and len(points[0]) > 1:
                    middle = (points[0][0] + points[0][1]) // 2
        else:              
            if point_count == 2:
                if len(points) > 0 and len(points[0]) > 1:
                    middle = (points[0][0] + points[0][1]) // 2
            elif point_count == 4:
                if len(points) > 1 and len(points[1]) > 1:
                    middle = (points[1][0] + points[1][1]) // 2
                




        # # average points
        # left = 0
        # middle = 0
        # right = 0
        # if len(left_points) > 0:
        #     left = sum(left_points) // len(left_points)
        # if len(middle_points) > 0:
        #     middle = sum(middle_points) // len(middle_points)
        # if len(right_points) > 0:
        #     right = sum(right_points) // len(right_points)
        # # print(f'middle: {middle - thresh.shape[1] // 2}')
        # # print(f'left: {left - thresh.shape[1] // 2} middle: {middle - thresh.shape[1] // 2} right: {right - thresh.shape[1] // 2}')
        
        # # print(f'left: {left} middle: {middle} right: {right}')
        
        # if len(points) > 0 and len(points[0]) > 1:
        #     middle = (points[0][0] + points[0][1]) // 2
            
        #     cv2.circle(frame_rgb, (points[0][0], start_height), 5, (0, 255, 0), 2)
        #     cv2.circle(frame_rgb, (points[0][1], start_height), 5, (0, 255, 0), 2)
        #     cv2.circle(frame_rgb, (middle, start_height), 5, (0, 0, 255), 2)
        #     print(f'middle: {middle - thresh.shape[1] // 2}')
            
        #     if len(points) > 1 and len(points[1]) > 1:
        #         middle = (points[1][0] + points[1][1]) // 2
                
        #         cv2.circle(frame_rgb, (points[1][0], start_height - 45), 5, (0, 255, 0), 2)
        #         cv2.circle(frame_rgb, (points[1][1], start_height - 45), 5, (0, 255, 0), 2)
        #         cv2.circle(frame_rgb, (middle, start_height - 45), 5, (0, 0, 255), 2)
        #         print(f'middle: {middle - thresh.shape[1] // 2}')
            
        # else:
        #     # start_height -= 10
        #     # start_height = start_height % h
        #     no_points_count += 1
        # # print(f'no_points_count: {no_points_count}')
            
        cv2.imshow('Video', frame_rgb)
        cv2.imshow('Videoth', thresh)
        # Press 'q' to exit
        if cv2.waitKey(60) & 0xFF == ord('q'):
            isProcessing = False
cap.release() 