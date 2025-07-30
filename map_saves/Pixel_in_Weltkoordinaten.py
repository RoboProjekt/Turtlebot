import imageio

# Konfiguration aus YAML
origin = [-12.4, -9.73]  # (x, y)
resolution = 0.05  # m/pixel
image_path = "/home/andy/turtlebot3_ws/map_saves/komplettes_Labor_mit_Flur.pgm"

x_Pixel = int(input("Bitte x_Pixel eingeben eingeben: "))
y_Pixel = int(input("Bitte y_Pixel eingeben: "))

# Bildgröße ermitteln
img = imageio.imread(image_path)
height = img.shape[0]

def pixel_to_world(px, py):
    wx = origin[0] + (px * resolution)
    wy = origin[1] + ((height - py) * resolution)
    return round(wx, 3), round(wy, 3)

print(pixel_to_world(x_Pixel,y_Pixel))
