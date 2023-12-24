# Segment Intersection Algorithm

#### There are no external libraries currently used

<ul>
<li> There are some code additions and comment additions I have done to check whether the efficient algorithm is working or not. </li>
<li> As I feel the all pairs intersection is throwing duplicates too I wrote a all_pairs_intersections_mod which filters out duplicates. </li>
<li> After using all_pairs_intersections_mod I was able to satisfy the condition and get the algorithm run a bit faster than the normal all paris intersection. </li>
<li> This code base is not using any custom libraries as such for handling BST or any sort of datastrcuture. Leveraging them can further reduce the time and efficiently handle the intersection and sorting but currently I haven't used it. </li>
<li> I have also added some code in comments to check sanity of efficient implementation on the test cases. For the test cases currently it gives all intersection points, but in case we want only a particular line1 vs line2, we can add one more condition to check whether the intersection point is present on the required line segment or not. </li> </ul>


### Code to check efficient algorithm implementation


        L1 = read_segments("random", 50)  
        L2 = read_segments("convex", 1)  
        points =  efficient_intersections(L1, L2)
        visualize_lines_and_intersections( L1, L2, points)

        L1 = read_segments("nonconvex", 11)  
        L2 = read_segments("convex", 3)  
        points =  efficient_intersections(L1, L2)
        visualize_lines_and_intersections( L1, L2, points)

        L1 = read_segments("nonconvex", 2)  
        L2 = read_segments("convex", 10)  
        points =  efficient_intersections(L1, L2)
        visualize_lines_and_intersections( L1, L2, points)



### Custom change in all_pairs_intersection



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
