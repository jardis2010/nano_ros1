#!/usr/bin/env python3
# encoding: utf-8
import cv2
import math
import numpy as np

def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离(distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)


def box_center(box):
    """
    计算四边形box的中心(calculate the center of quadrangle box)
    :param box: box （x1, y1, x2, y2)形式(box （x1, y1, x2, y2)type)
    :return:  中心坐标（x, y)(center coordinate（x, y))
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def bgr8_to_jpeg(value, quality=75):
    """
    将cv bgr8格式数据转换为jpg格式(convert data in the format of cv bgr8 into jpg)
    :param value: 原始数据(original data)
    :param quality:  jpg质量 最大值100(jpg quality. Maximum value is 100)
    :return:
    """
    return bytes(cv2.imencode('.jpg', value)[1])


def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from a picture to a new picture of different size)
    :param point: 点的坐标(coordinate of point)
    :param now: 现在图片的尺寸(size of current picture)
    :param new: 新的图片尺寸(new picture size)
    :return: 新的点坐标(new point coordinate)
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)


def get_area_max_contour(contours, threshold=100):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the contour whose area is the largest. Filter out those whose area is too small)
    :param contours: 轮廓列表(contour list)
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(area threshold. Contour whose area is less than this value will be filtered out)
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the maximum contour area is greater than this threshold, return the largest contour, otherwise return None)
    """
    contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
    if len(contour_area) > 0:
        max_c_a = max(contour_area, key=lambda c_a: c_a[1])
        return max_c_a
    return None


def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi(calculate the angle between two vectors -pi ~ pi)
    :param v1: 第一个向量(first vector)
    :param v2: 第二个向量(second vector)
    :return: 角度(angle)
    """
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (d_v1_v2)
    sin = np.cross(v1, v2) / (d_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
    简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平(Simple alignment. Calculate the angle of the line connecting the two points. Rotate the picture around the image center to make the line horizontal)
    可以用在人脸对齐上(can be used to align the face)

    :param image: 要选择的人脸图片(select face picture)
    :param points: 两个点的坐标 ((x1, y1), (x2, y2))(coordinate of two points ((x1, y1), (x2, y2)))
    :param scale: 缩放比例(scaling)
    :return: 旋转后的图片(rotated picture)
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # 计算旋转角度并旋转图片(calculate the rotation angle and rotate picture)
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))
