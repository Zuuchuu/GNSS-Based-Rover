import tkinter as tk
from tkinter import ttk
import webbrowser
import webview


def create_map_window(latitude, longitude):
    
    url = f"https://www.google.com/maps?q={latitude},{longitude}"
    map_window = webview.create_window("GPS Location Viewer", url)
    webview.start()

def on_show_map():
    latitude = 10.77197  
    longitude = 106.65124  
    create_map_window(latitude, longitude)


def main():
    root = tk.Tk()
    root.title("Google Maps Viewer")

    map_frame = ttk.Frame(root, padding=10)
    map_frame.pack(fill=tk.BOTH, expand=True)

    
    map_frame.grid_columnconfigure(0, weight=1)
    map_frame.grid_rowconfigure(0, weight=1)

    
    map_image_path = "map.png"
    map_image = tk.PhotoImage(file=map_image_path)
    map_label = tk.Label(map_frame, image=map_image)
    map_label.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

    
    map_button = tk.Button(map_frame, text='Show map', width=15, command=on_show_map, bg="green", fg="white", font=("Times New Roman", 12, "bold"))
    map_button.grid(row=1, column=0, pady=10)

    root.mainloop()

if __name__ == "__main__":
    main()
