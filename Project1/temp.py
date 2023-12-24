    def bentley_ottman_two_sets(segments1, segments2):


        events = []

        for i, ((x1, y1), (x2, y2)) in enumerate(segments1):
            if (x1, y1) > (x2, y2):
                x1, y1, x2, y2 = x2, y2, x1, y1
            events.append((x1, ((x1, y1), (x2, y2)), "left"))
            events.append((x2, ((x1, y1), (x2, y2)), "right"))

        for i, ((x1, y1), (x2, y2)) in enumerate(segments2):
            if (x1, y1) > (x2, y2):
                x1, y1, x2, y2 = x2, y2, x1, y1
            events.append((x1, ((x1, y1), (x2, y2)), "left2"))
            events.append((x2, ((x1, y1), (x2, y2)), "right2"))

        events.sort()

        sweep_line = []
        intersections = []

        for event in events:
            x, segment, event_type = event

            if event_type == "left":
                for seg in sweep_line:
                    intersection = find_intersection(segment, seg)
                    if intersection is not None:
                        intersections.append(intersection)
                sweep_line.append(segment)
                sweep_line.sort(key=lambda s: s[0][1])
            elif event_type == "right":
                sweep_line.remove(segment)
            elif event_type == "left2":
                for seg in sweep_line:
                    intersection = find_intersection(segment, seg)
                    if intersection is not None:
                        intersections.append(intersection)
                sweep_line.append(segment)
                sweep_line.sort(key=lambda s: s[0][1])
            else:
                sweep_line.remove(segment)

        # Remove duplicate intersection points
        unique_intersections = list(set(intersections))
        return intersections
    
    # Usage with one set of line segments
    LK1 = L1