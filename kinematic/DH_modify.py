import numpy as np
def DH_modify(alpha,a,theta,d,degree):
    """
    Tính ma trận biến đổi đồng nhất sử dụng tham số Denavit-Hartenberg đã được chỉnh sửa.

    Tham số:
    theta: Góc quay quanh trục z (radian)
    d: Dịch chuyển dọc theo trục z (mm)
    a: Dịch chuyển dọc theo trục x (mm)
    alpha: Góc quay quanh trục x (radian)

    Trả về:
    Ma trận 4x4 biểu diễn biến đổi đồng nhất.
    """
    if degree:
        theta = np.radians(theta)
        alpha = np.radians(alpha)
    else:
        theta = theta
        alpha = alpha
    T= np.array([[np.cos(theta),-np.sin(theta),0,a],
                 [np.sin(theta)*np.cos(alpha),np.cos(theta)*np.cos(alpha),-np.sin(alpha),-np.sin(alpha)*d],
                 [np.sin(theta)*np.sin(alpha),np.cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d],
                 [0,0,0,1]])
    return T