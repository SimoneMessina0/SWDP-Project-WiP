import numpy as np
import matplotlib.pyplot as plt
import serial
import os
import pandas as pd
from datetime import datetime
from tkinter import Tk, filedialog, ttk, messagebox, StringVar, Label, Button
from serial.tools import list_ports

def convert_16bit_signed(lo, hi):
    combined = (hi.astype(np.uint16) << 8) | lo.astype(np.uint16)
    return combined.view(np.int16)

def conv_imu(arr):
    x = convert_16bit_signed(arr[:,0], arr[:,1]) / (2**15) * 2
    y = convert_16bit_signed(arr[:,2], arr[:,3]) / (2**15) * 2
    z = convert_16bit_signed(arr[:,4], arr[:,5]) / (2**15) * 2
    return x, y, z

def conv_gyro(arr):
    x = convert_16bit_signed(arr[:,0], arr[:,1]) / (2**15) * 250
    y = convert_16bit_signed(arr[:,2], arr[:,3]) / (2**15) * 250
    z = convert_16bit_signed(arr[:,4], arr[:,5]) / (2**15) * 250
    return x, y, z

def receive_and_save_data(ser, bin_filename, packet_size=4096, max_packets=2048*64):
    """Riceve pacchetti via seriale e li salva in binario."""
    with open(bin_filename, 'wb') as f:
        for packet_count in range(max_packets):
            data = ser.read(packet_size)
            if not data:
                continue
            if data == b'T':  # terminatore
                print("Received Terminator Character.")
                break
            f.write(data)
            print(f"📥 Received packet {packet_count+1}")
    ser.close()
    print("📴 Serial COM Port Closed.")

def process_bin_file(bin_filename, csv_filename=None):
    hh_list, mm_list, ss_list, sss_list = [], [], [], []
    acc_x_list, acc_y_list, acc_z_list = [], [], []
    gyro_x_list, gyro_y_list, gyro_z_list = [], [], []
    ppg0_list, ppg1_list = [], []
    temp_list = []

    # Each NAND page is sent as 4096 bytes. We ignore the last 16 spare bytes
    # and parse subpackets. Each sample contains: 5 bytes timestamp, 6 bytes
    # accelerometer, 6 bytes gyroscope, 6 bytes MAX30101 (3 bytes per LED)
    SUBPKT_SIZE = 25
    with open(bin_filename, "rb") as f:
        while True:
            pagina = f.read(4096)
            if len(pagina) < 4096:
                break
            # consider only the first 4080 bytes (4096 - 16 spare bytes)
            valid_bytes = pagina[:4080]
            # parse subpackets of SUBPKT_SIZE bytes
            for i in range(0, len(valid_bytes), SUBPKT_SIZE):
                subpkt = valid_bytes[i:i+SUBPKT_SIZE]
                if len(subpkt) < SUBPKT_SIZE:
                    continue
                # timestamp
                hh = subpkt[0]
                mm = subpkt[1]
                ss = subpkt[2]
                sss = subpkt[3] | (subpkt[4] << 8)
                hh_list.append(hh)
                mm_list.append(mm)
                ss_list.append(ss)
                sss_list.append(sss)
                # dati IMU
                # accelerometer: bytes 5..10 (6 bytes)
                acc_arr = np.frombuffer(subpkt[5:11], dtype=np.uint8).reshape(1,6)
                acc_x, acc_y, acc_z = conv_imu(acc_arr)
                # gyroscope: bytes 11..16 (6 bytes)
                gyro_arr = np.frombuffer(subpkt[11:17], dtype=np.uint8).reshape(1,6)
                gx, gy, gz = conv_gyro(gyro_arr)
                acc_x_list.append(acc_x[0])
                acc_y_list.append(acc_y[0])
                acc_z_list.append(acc_z[0])
                gyro_x_list.append(gx[0]) 
                gyro_y_list.append(gy[0])
                gyro_z_list.append(gz[0])
                # PPG (MAX30101): two 24-bit big-endian values
                ppg0 = int.from_bytes(subpkt[17:20], byteorder='big') >> 3  # shift di 3 per formato a 15 bit
                ppg1 = int.from_bytes(subpkt[20:23], byteorder='big') >> 3  # shift di 3 per formato a 15 bit
                temp = int.from_bytes(subpkt[23:25], byteorder='big')
                ppg0_list.append(ppg0)
                ppg1_list.append(ppg1)
                temp_list.append(temp)

    # crea DataFrame
    df = pd.DataFrame({
        "hh": hh_list,
        "mm": mm_list,
        "ss": ss_list,
        "sss": sss_list,
        "acc_x": acc_x_list,
        "acc_y": acc_y_list,
        "acc_z": acc_z_list,
        "gyro_x": gyro_x_list,
        "gyro_y": gyro_y_list,
        "gyro_z": gyro_z_list,
        "ppg_led0": ppg0_list,
        "ppg_led1": ppg1_list,
        "skin_temperature": [temp * 0.00390625 for temp in temp_list]
    })

    # plot accelerometro
    plt.figure(figsize=(15,5))
    plt.subplot(2,1,1)
    plt.plot(df.index, df["acc_x"], label="acc_x")
    plt.plot(df.index, df["acc_y"], label="acc_y")
    plt.plot(df.index, df["acc_z"], label="acc_z")
    plt.title("Accelerometer")
    plt.xlabel("Subpacket index")
    plt.ylabel("g")
    plt.legend()
    plt.grid(True)

    # plot giroscopio
    plt.subplot(2,1,2)
    plt.plot(df.index, df["gyro_x"], label="gyro_x")
    plt.plot(df.index, df["gyro_y"], label="gyro_y")
    plt.plot(df.index, df["gyro_z"], label="gyro_z")
    plt.title("Gyroscope")
    plt.xlabel("Subpacket index")
    plt.ylabel("deg/s")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # plot PPG (MAX30101)
    fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(15,5))

    axs[0].plot(df.index, df["ppg_led0"], label="ppg_led0")
    axs[1].plot(df.index, df["ppg_led1"], label="ppg_led1")
    fig.suptitle("MAX30101 PPG")
    axs[1].set_xlabel("Subpacket index")
    axs[0].set_ylabel("Raw 24-bit value")
    axs[1].set_ylabel("Raw 24-bit value")
    axs[0].legend()
    axs[1].legend()
    axs[0].grid(True)
    axs[1].grid(True)
    plt.tight_layout()
    plt.show()

    # plot skin temperature (MAX30205)
    plt.figure(figsize=(15,5))
    plt.plot(df.index, df["skin_temperature"], label="skin_temperature", color="tab:red")
    plt.title("Skin Temperature")
    plt.xlabel("Subpacket index")
    plt.ylabel("°C")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # salva CSV
    if csv_filename:
        df.to_csv(csv_filename, index=False)
        print(f"📄 Data saved in CSV: {csv_filename}")

def gui_select_com_and_folder():
    """Apre una piccola GUI per selezionare COM e cartella."""
    root = Tk()
    root.title("IMU Data Logger - Configuration")
    root.geometry("400x400")
    root.resizable(False, False)

    Label(root, text="🔌 Select the COM Port:", font=("Segoe UI", 10)).pack(pady=5)
    
    com_var = StringVar()
    ports = [p.device for p in list_ports.comports()]

    if not ports:
        ports = ["No COM Port found"]
    com_box = ttk.Combobox(root, textvariable=com_var, values=ports, state="readonly", width=30)
    com_box.pack(pady=5)
    com_box.current(0)

    def browse_folder():
        folder = filedialog.askdirectory(title="📂 Select the folder")
        if folder:
            folder_var.set(folder)

    folder_var = StringVar()
    Label(root, text="📁 Folder:", font=("Segoe UI", 10)).pack(pady=5)
    Button(root, text="Select folder...", command=browse_folder).pack()
    Label(root, textvariable=folder_var, fg="blue", wraplength=350).pack(pady=5)

    def confirm():
        if not folder_var.get() or "Nessuna" in com_var.get():
            messagebox.showerror("Error", "Select a valid COM Port and a folder.")
            return
        root.destroy()

    Button(root, text="✅ Confirm", command=confirm, bg="#4CAF50", fg="white").pack(pady=10)
    root.mainloop()

    return com_var.get(), folder_var.get()

# ==============================
# Main
# ==============================

def main():
    com_port, save_path = gui_select_com_and_folder()
    if not com_port or not save_path:
        print("❌ Application Stopped.")
        return

    base_filename = datetime.now().strftime("IMUData_%Y%m%d_%H%M%S")
    bin_filename = os.path.join(save_path, f"{base_filename}.bin")
    csv_filename = os.path.join(save_path, f"{base_filename}_imu.csv")

    BAUD_RATE = 250000

    try:
        ser = serial.Serial(com_port, BAUD_RATE, timeout=10)
        print(f"🔌 Connected to {com_port}")
        receive_and_save_data(ser, bin_filename)
    except serial.SerialException as e:
        print(f"⚠️ Serial Error: {e}")
        return

    process_bin_file(bin_filename, csv_filename)

# ==============================
# Entry point
# ==============================

if __name__ == "__main__":
    main()