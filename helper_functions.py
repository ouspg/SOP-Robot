from math import *

def sphere_distance(a, b): #Input in sphere coordinates
    a_x, a_y, a_z = to_cartesian(a)
    b_x, b_y, b_z = to_cartesian(b)

    r = sqrt(pow(a_x - b_x, 2) + pow(a_y - b_y, 2) + pow(a_z - b_z, 2))
    return r

def to_cartesian(sp_coord): #[Inclination, Azimuth, Radius]/[i, a, r]
    i, a, r = sp_coord

    x = r * sin(i) * cos(a)
    y = r * sin(i) * cos(a)
    z = r * cos(i)
    return (x, y, z)

def to_sphere(ca_coord): #[X, Y, Z]
    x, y, z = ca_coord

    r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))
    a = atan(y / x)
    i = acos(z / r)
    return (i, a, r)

def get_angles(x, y, img_w, img_h, cam_fov):
    x_norm = translate(x, 0, img_w, -1, 1)
    y_norm = translate(y, 0, img_h, -1, 1)

    return [degrees(tan(x_norm * atan(radians(cam_fov[0])))), -degrees(tan(y_norm * atan(radians(cam_fov[1]))))]
    #return [((x - img_w / 2)/ img_w) * (cam_fov[0] / 2), ((y - img_h / 2)/ img_h) * (cam_fov[1] / 2)]

def get_angle_width(f_w, f_h, w, h, cam_fov):
    return f_w / w * cam_fov[0]

def get_distance(w, h, img_w, img_h, cam_fov):
    human_face = [0.15, 0.15] #Estimated width of a persons face also accounting for the slightly larger detection area created by the classifier
    angle_w = get_angle_width(w, h, img_w, img_h, cam_fov)
    distance = human_face[0] / atan(radians(angle_w))
    return distance

def get_center(x, y, h, w):
    return [int((2 * x + w) / 2), int((2 * y + h) / 2)]

#From "https://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another"
def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)
