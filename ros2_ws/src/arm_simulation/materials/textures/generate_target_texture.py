from PIL import Image, ImageDraw, ImageFont
import os

def main():
    # Dimensiones
    size = 512
    
    # Intentar cargar una fuente sans-serif del sistema
    font_paths = [
        "/usr/share/fonts/TTF/DejaVuSans-ExtraLight.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/LiberationSans-Regular.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/TTF/LiberationSans-Bold.ttf",
        "/usr/share/fonts/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    ]
    
    font = None
    loaded_path = None
    for path in font_paths:
        if os.path.exists(path):
            try:
                # Reducimos el tamaño a 160 (antes era probablemente mucho más grande, como 350-400)
                font = ImageFont.truetype(path, 160)
                loaded_path = path
                break
            except Exception:
                pass
                
    if font is None:
        font = ImageFont.load_default()
        print("Usando fuente por defecto")

    # Letras requeridas
    letters = ["A", "D", "R", "I", "N", "E", "O"]
    
    script_dir = "/home/reve/Documents/UC/robotica/finalProject/ros2_ws/src/arm_simulation/materials/textures"
    install_dir = "/home/reve/Documents/UC/robotica/finalProject/ros2_ws/install/arm_simulation/share/arm_simulation/materials/textures"
    os.makedirs(script_dir, exist_ok=True)
    os.makedirs(install_dir, exist_ok=True)

    for letter in letters:
        # Crear fondo cian
        img = Image.new("RGB", (size, size), color=(0, 204, 204))
        draw = ImageDraw.Draw(img)
        
        # Obtener el tamaño del texto para centrarlo
        try:
            bbox = draw.textbbox((0, 0), letter, font=font)
            w = bbox[2] - bbox[0]
            h = bbox[3] - bbox[1]
        except AttributeError:
            w, h = draw.textsize(letter, font=font)
            
        x = (size - w) / 2
        y = (size - h) / 2 - 20 # Ajuste fino vertical
        
        draw.text((x, y), letter, fill=(0, 0, 0), font=font)
        
        # Rotar 90 grados en sentido horario
        rotated_img = img.transpose(Image.ROTATE_270)
        
        filename = f"target_draw_{letter}.png"
        
        # Guardar en src
        rotated_img.save(os.path.join(script_dir, filename), format="PNG")
        print(f"Textura guardada: {os.path.join(script_dir, filename)}")
        
        # Guardar en install
        install_path = os.path.join(install_dir, filename)
        if os.path.islink(install_path):
            os.unlink(install_path)
        rotated_img.save(install_path, format="PNG")
        print(f"Textura guardada: {install_path}")
        
        # Guardar también el fallback target_draw.png como la letra A
        if letter == "A":
            rotated_img.save(os.path.join(script_dir, "target_draw.png"), format="PNG")
            
            inst_fallback = os.path.join(install_dir, "target_draw.png")
            if os.path.islink(inst_fallback):
                os.unlink(inst_fallback)
            rotated_img.save(inst_fallback, format="PNG")

if __name__ == "__main__":
    main()
