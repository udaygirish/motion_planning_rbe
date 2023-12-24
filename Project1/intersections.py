import time

def segment_pair_intersection(line1, line2):
    '''Return the intersection point between 2 line segments or None, if they do not intersect. 

    arguments:
    line1 - is the first line segment and has the following form ((x1,y1), (x2,y2)) where x1, y1 is
      the first point of the line segment and x2,y2 is the second point of the line segment.
    line2 - is the second line segment and has the same format as line1.

    return:
    ipoint - A tuple (x,y) which is the point of intersection or None if it does not exist 
    '''
    ### YOUR CODE STARTS HERE ###
    #First line calculation - Form a1*x + b1*y = c1
    a1 = line1[0][1] - line1[1][1]
    b1 = line1[1][0] - line1[0][0]
    c1 = line1[0][0]*a1 + line1[0][1]*b1

    # Second line calculation - Form a2*x + b2*y = c2
    a2 = line2[0][1] - line2[1][1]
    b2 = line2[1][0] - line2[0][0]
    c2 = line2[0][0]*a2 + line2[0][1]*b2

    # After converting to system of linear equation with matrix form AX=B
    detA = b2*a1 - a2*b1

    # Check whether the determinant is non zero or not
    if detA != 0:
        intersect = True
    else:
        intersect = False

    if intersect:
        # intersection point between line1 and line2
        (inter_x,inter_y) = ((b2*c1-b1*c2)/detA, (c2*a1-c1*a2)/detA)
        list_x_1 = [line1[0][0], line1[1][0]]
        list_x_2 = [line2[0][0], line2[1][0]]
        list_y_1 = [line1[0][1], line1[1][1]]
        list_y_2 = [line2[0][1], line2[1][1]]
        ep = 5e-10 # Managing precision can be increased or decreased 
        # To ensure we are not missing point which are exactly on the line
        # but to a higher precision value

        # Adding range based conditions to avoid intersection points external to the endpoints
        cond_x_1 = min(list_x_1) - ep <= inter_x <= max(list_x_1) +ep
        cond_x_2 = min(list_x_2) -ep <= inter_x <= max(list_x_2) +ep
        cond_y_1 = min(list_y_1) -ep <= inter_y <= max(list_y_1) +ep
        cond_y_2 = min(list_y_2) -ep <= inter_y <= max(list_y_2) +ep
        # Check all the range conditions are satisfied or not
        if cond_x_1 and cond_x_2 and cond_y_1 and cond_y_2:
            return (inter_x, inter_y)
        else:
            return None
    else:
        return None

    ### YOUR CODE ENDS HERE ###

def efficient_intersections(L1, L2):
    #This is the Bonus part of the assignment 
    #Calculate the intersection point using line equations.

    ## YOUR CODE STARTS HERE ###
    def find_intersection(line1, line2):
            # The below code is same as the two point intersection above 
            # This is a algorithm which is having time complexity of O(1)
            a1 = line1[0][1] - line1[1][1]
            b1 = line1[1][0] - line1[0][0]
            c1 = line1[0][0]*a1 + line1[0][1]*b1

            # Second line calculation - Form a2*x + b2*y = c2
            a2 = line2[0][1] - line2[1][1]
            b2 = line2[1][0] - line2[0][0]
            c2 = line2[0][0]*a2 + line2[0][1]*b2

            # After converting to system of linear equation with matrix form AX=B
            detA = b2*a1 - a2*b1

            # print("Determinant")
            # print(detA)
            if detA != 0:
                intersect = True
            else:
                intersect = False

            if intersect:
                # intersection point between line1 and line2
                (inter_x,inter_y) = ((b2*c1-b1*c2)/detA, (c2*a1-c1*a2)/detA)
                list_x_1 = [line1[0][0], line1[1][0]]
                list_x_2 = [line2[0][0], line2[1][0]]
                list_y_1 = [line1[0][1], line1[1][1]]
                list_y_2 = [line2[0][1], line2[1][1]]
                ep = 5e-10 # Managing precision can be increased or decreased 
                # To ensure we are not missing point which are exactly on the line
                # but to a higher precision value
                cond_x_1 = min(list_x_1) - ep <= inter_x <= max(list_x_1) +ep
                cond_x_2 = min(list_x_2) -ep <= inter_x <= max(list_x_2) +ep
                cond_y_1 = min(list_y_1) -ep <= inter_y <= max(list_y_1) +ep
                cond_y_2 = min(list_y_2) -ep <= inter_y <= max(list_y_2) +ep
                if cond_x_1 and cond_x_2 and cond_y_1 and cond_y_2:
                    return (inter_x, inter_y)
                else:
                    return None
            else:
                return None
            
    def bentley_ottman_sweepl(line_segments):
        # Initialize events - for adding the endpoints
        events = []

        # Enumerating the coordinates from line_segments
        for i, ((x1, y1), (x2, y2)) in enumerate(line_segments):
            # Ordering by max tuple
            if (x1, y1) > (x2, y2):
                # Swapping values
                x1, y1, x2, y2 = x2, y2, x1, y1
            # Once done assign left and right endpoints
            events.append((x1, ((x1, y1), (x2, y2)), "left"))
            events.append((x2, ((x1, y1), (x2, y2)), "right"))

        # Sort the events according from top to bottom
        events.sort()

        # Initialize sweep line and intersections_x and intersections_y and intersections
        sweep_line = []
        intersections_x = []
        intersections_y = []
        intersections = []

        # Iterate through events
        for event in events:

            # Get the left coordinate and the segment and whether it is left or right endpoint type
            # Or if none it can be a intermediate intersection point
            x, segment, event_type = event

            # If event type is left 
            if event_type == "left":

                # Find all intersections around neighbours
                for seg in sweep_line:
                    intersection = find_intersection(segment, seg)
                    # If there is a intersection append the intersection to the total list
                    if intersection is not None:
                        intersections.append(intersection)
                # Add the segment to the sweep line
                sweep_line.append(segment)
                # Sort the list according to y key
                sweep_line.sort(key=lambda s: s[0][1])
            elif event_type == "right":
                # If the event_type is right it means we have reached the right endpoint
                # here we can remove the segment as it is not needed for further analysis.
                sweep_line.remove(segment)
            else:
                # If the event_type is not both left and right and still it is a event
                # then this can be the case of intermediate intersection point
                i = sweep_line.index(segment)
                # Here we swap the segments order
                sweep_line[i], sweep_line[i + 1] = sweep_line[i + 1], sweep_line[i]

                # Once done get the new neighbours
                above = sweep_line[i - 1]
                below = sweep_line[i + 1]

                # If above neighbour exists
                if above is not None:
                    # If the above neighbour is having intersection append the intersection
                    intersection = find_intersection(segment, above)
                    if intersection is not None:
                        intersections.append(intersection)

                # If below neighbour exists
                if below is not None:
                    # If the below neighbour is having intersection append it
                    intersection = find_intersection(segment, below)
                    if intersection is not None:
                        intersections.append(intersection)

        # After getting over intersections - Check for duplicates Possible in case we deal with collinear segments
        intersections = list(set(intersections))
        # Get x and y intersection points separately.
        intersections_x = [i[0] for i in intersections]
        intersections_y = [i[1] for i in intersections]
        return intersections_x, intersections_y 
    
    # Get all the line segments - Combine both lists
    # In sweep line we use a single list we can do with two segments but combining is more efficient
    L_final = L1+L2
    # Remove duplicates segments if any
    L_final = list(set(L_final))
    # Add condition to check if the length is zero to avoid no segment condition
    if len(L_final) >0:
        # Call Bentley Ottman algorithm for one segment list
        intersections_x, intersections_y = bentley_ottman_sweepl(L_final)
        return (intersections_x, intersections_y)
    else:
        return ([],[])
    
    
    ### YOUR CODE ENDS HERE ###

def all_pairs_intersections(L1, L2):
    x = []
    y = []
    for l1 in L1: 
        for l2 in L2: 
          point =  segment_pair_intersection(l1, l2)
          if point: 
            x.append(point[0])
            y.append(point[1])

    return (x,y)

# CUSTOM ALL PAIRS INTERSECTION
# Use the below to eliminate duplicates to get the correct result as the above allows duplicates 
# This implementation I added custom to check whether duplicates are present or not and the assumption is correct.
def all_pairs_intersections_mod(L1, L2):
    x = []
    y = []
    points = []
    for l1 in L1: 
        for l2 in L2: 
          point =  segment_pair_intersection(l1, l2)
          if point:
            points.append(point)
    points = list(set(points))
    if len(points) >0:    
        x = [i[0] for i in points]
        y = [i[1] for i in points]

    return (x,y)

# To check efficient implementation please add the below to the main code
# This can check whether the algorithm is able to find all intersections of L1 or not
# In Pairwise intersection we used the same L1 and L2 as L1
# This is added by me to check the Efficient algorithm
# L1 = read_segments("random", 50)  
# L2 = read_segments("convex", 1)  
# points =  efficient_intersections(L1, L2)
# visualize_lines_and_intersections( L1, L2, points)

# L1 = read_segments("nonconvex", 11)  
# L2 = read_segments("convex", 3)  
# points =  efficient_intersections(L1, L2)
# visualize_lines_and_intersections( L1, L2, points)

# L1 = read_segments("nonconvex", 2)  
# L2 = read_segments("convex", 10)  
# points =  efficient_intersections(L1, L2)
# visualize_lines_and_intersections( L1, L2, points)