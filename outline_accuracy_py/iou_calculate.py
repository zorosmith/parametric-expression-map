import random
import time
from sympy.geometry import *
import numpy as np
from scipy import stats
 
 
def polygon_over_area(obj_a, obj_b):
    p_a = obj_a  # sympy.geometry.Polygon
    p_b = obj_b
    x_obj = p_a.intersection(p_b)
    if not x_obj:
        if p_b.encloses_point(p_a.vertices[0]):
            return p_a.area
        if p_a.encloses_point(p_b.vertices[0]):
            return p_b.area
        print('No intersection between groundtruth plane and estimation plane')
        return 0
 
    x_point = []
    segment_list = []
    for item in x_obj:
        if isinstance(item, Point):
            x_point.append(item)
        elif isinstance(item, Segment):
            for p in item.points:
                if p in x_point:
                    x_point.remove(p)
                else:
                    x_point.append(p)
            segment_list.append(item)
 
    region = []
    a_point_in_b = []
    for v in p_a.vertices:
        if p_b.encloses_point(v):
            a_point_in_b.append(v)
    b_point_in_a = []
    for v in p_b.vertices:
        if p_a.encloses_point(v):
            b_point_in_a.append(v)
 
    new_region_point = a_point_in_b + b_point_in_a + x_point
    over_area = 0
    while 1:
        if segment_list:
            while 1:
                if not segment_list:
                    select_segment = None
                    break
                select_segment = segment_list.pop()
                verify_point = select_segment.points[0]
                point_segment = select_segment
                need_continue = False
                while 1:
                    if verify_point not in x_point:
                        for segment in segment_list:
                            if segment == point_segment:
                                continue
                            if segment.contains(verify_point):
                                point_segment = segment
                                if segment.points[1] == verify_point:
                                    verify_point = segment.points[0]
                                else:
                                    verify_point = segment.points[1]
                                break
                        else:
                            need_continue = True
                            break
                    else:
                        break
                if need_continue:
                    continue
                for side in p_a.sides:
                    if not side.contains(select_segment.points[0]):
                        continue
                    if not side.contains(select_segment.points[1]):
                        continue
                    a_side = side
                    break
                else:
                    raise ValueError
                for side in p_b.sides:
                    if not side.contains(select_segment.points[0]):
                        continue
                    if not side.contains(select_segment.points[1]):
                        continue
                    b_side = side
                    break
                else:
                    raise ValueError
                v_a = a_side.points[1] - a_side.points[0]
                v_b = b_side.points[1] - b_side.points[0]
                if (v_a[0] * v_b[0] < 0) or (v_a[1] * v_b[1] < 0):
                    continue
                if select_segment.points[0] == a_side.points[1]:
                    end_point = a_side.points[1]
                elif select_segment.points[0] == b_side.points[1]:
                    end_point = b_side.points[1]
                elif select_segment.points[1] == a_side.points[1]:
                    end_point = a_side.points[1]
                elif select_segment.points[1] == b_side.points[1]:
                    end_point = b_side.points[1]
                else:
                    raise ValueError
                while 1:
                    for segment in segment_list:
                        if segment.contains(end_point):
                            end_point = segment.points[0] if end_point == segment.points[1] else segment.points[1]
                            segment_list.remove(segment)
                            break
                    else:
                        break
                point_line_a = []
                point_line_b = []
                for side in p_a.sides:
                    if side.contains(end_point):
                        point_line_a.append(side)
                for side in p_b.sides:
                    if side.contains(end_point):
                        point_line_b.append(side)
                if len(point_line_a) == 2:
                    if len(point_line_b) != 1:
                        raise ValueError
                    select_inner_point = end_point
                    select_polygon = p_a
                    next_polygon = p_b
                    break
                elif len(point_line_b) == 2:
                    if len(point_line_a) != 1:
                        raise ValueError
                    select_inner_point = end_point
                    select_polygon = p_b
                    next_polygon = p_a
                    break
                else:
                    raise ValueError
            if not select_segment:
                continue
        elif a_point_in_b:
            select_inner_point = a_point_in_b.pop()
            select_polygon = p_a
            next_polygon = p_b
        elif b_point_in_a:
            select_inner_point = b_point_in_a.pop()
            select_polygon = p_b
            next_polygon = p_a
        else:
            break
        new_region_points = [select_inner_point]
        while 1:
            for side in select_polygon.sides:
                if side.contains(select_inner_point) and side.points[1] != select_inner_point:
                    select_side = Segment(select_inner_point, side.points[1])
                    break
            else:
                raise ValueError
            x_p_list = []
            for x_p in x_point:
                if select_side.contains(x_p):
                    if select_inner_point == x_p:
                        continue
                    x_p_list.append(x_p)
 
            if x_p_list:
                area = select_inner_point.distance(x_p_list[0])
                next_point = x_p_list[0]
                for x_p in x_p_list:
                    if 0 < select_inner_point.distance(x_p) < area:
                        area = select_inner_point.distance(x_p)
                        next_point = x_p
                select_inner_point = next_point
                x_point.remove(next_point)
                select_polygon, next_polygon = next_polygon, select_polygon
            else:
                select_inner_point = select_side.points[1]
            if select_inner_point in a_point_in_b:
                a_point_in_b.remove(select_inner_point)
            if select_inner_point in b_point_in_a:
                b_point_in_a.remove(select_inner_point)
            if select_inner_point in new_region_points:
                over_area += Polygon(*new_region_points).area
                break
            else:
                new_region_points.append(select_inner_point)
 
    return over_area

def main():
    P1 = Point(0,0)
    P2 = Point(4,0)
    P3 = Point(4,4)
    P4 = Point(0,4)
    P5 = Point(1,1)
    P6 = Point(5,1)
    P7 = Point(5,5)
    P8 = Point(1,5)
    poly_groundtruth = Polygon(P1,P2,P3,P4)
    poly_estimation = Polygon(P5,P6,P7,P8)
    intersection_area = polygon_over_area(poly_groundtruth, poly_estimation)
    print ('intersection area: ', intersection_area)
    gt_area = 16.0   # float
    est_area = 16.0   # float

    TP = float(intersection_area)  # float
    FP = est_area - TP
    FN = gt_area - TP
     
    Branching_factor = FP/TP
    Miss_factor = FN/TP
    Detection_percentage = 100*(TP/(TP+FN))
    Quality_percentafe = 100*(TP/(TP+FP+FN))
    print ('Branching_factor: ', Branching_factor)
    print ('Miss factor: ', Miss_factor)
    print ('Detection_percentage: ', Detection_percentage,'%')
    print ('Quality_percentafe: ', Quality_percentafe,'%')

if __name__ == '__main__':
    main()
    print("OK")
