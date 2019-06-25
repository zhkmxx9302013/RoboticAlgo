import cv2 as cv

def get_map():
    """
    获取地图
    :return:
    """
    np_map = cv.imread('map/1.bmp')
    # print(np_map)
    return np_map

# if __name__ == '__main__':
#     get_map()