import asyncio
from aiohttp import web
import matplotlib.pyplot as plt
from datetime import datetime
import re
import threading

# Initialize lists to store received data points and timestamps for each SDV
speed_data_sdv1 = []
speed_data_sdv2 = []
timestamps = []

# Initialize the plot figure and axis
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(figsize=(10, 6))
line_sdv1, = ax.plot([], [], marker='o', linestyle='-', color='b', label="SDV1 Speed")
line_sdv2, = ax.plot([], [], marker='o', linestyle='-', color='r', label="SDV2 Speed")
ax.set_title("SDV1 and SDV2 Speed Data")
ax.set_xlabel("Time (sec)")
ax.set_ylabel("Speed (km/h)")
ax.grid(True)
ax.legend()

# Function to update the plot with new data
def update_plot():
    # Update SDV1 data
    line_sdv1.set_xdata(range(len(speed_data_sdv1)))  # Use index for x-axis
    line_sdv1.set_ydata(speed_data_sdv1)
    
    # Update SDV2 data
    line_sdv2.set_xdata(range(len(speed_data_sdv2)))  # Use index for x-axis
    line_sdv2.set_ydata(speed_data_sdv2)
    
    ax.relim()         # Recompute the limits based on the new data
    ax.autoscale_view()  # Rescale the view to fit the new data
    plt.draw()
    plt.pause(0.1)  # Brief pause to update the plot visually

# Define an asynchronous function to handle incoming requests
async def handle_request(request):
    global speed_data_sdv1, speed_data_sdv2, timestamps
    
    # Read the text from the incoming request
    message = await request.text()
    print("Received message:", message)
    
    # Extract speed value based on whether message contains SDV1 or SDV2
    if "SDV1" in message:
        match = re.search(r"SDV1 speed is (\d+)km/h", message)
        if match:
            speed_value = float(match.group(1))
            speed_data_sdv1.append(speed_value)
            timestamps.append(datetime.now().strftime('%H:%M:%S'))
            print(f"Extracted speed for SDV1: {speed_value} km/h")
    elif "SDV2" in message:
        match = re.search(r"SDV2 speed is (\d+)km/h", message)
        if match:
            speed_value = float(match.group(1))
            speed_data_sdv2.append(speed_value)
            timestamps.append(datetime.now().strftime('%H:%M:%S'))
            print(f"Extracted speed for SDV2: {speed_value} km/h")
    else:
        print("No valid SDV data found in message.")

    # Return a response indicating that the message was received
    return web.Response(text="Message received", status=200)

# Function to continuously update the plot in a separate thread
def plot_updater():
    while True:
        if speed_data_sdv1 or speed_data_sdv2:
            update_plot()
        plt.pause(1)  # Adjust pause to control update frequency

# Define an asynchronous function to initialize the web application
async def init_app():
    # Start the plot updater in a separate thread
    threading.Thread(target=plot_updater, daemon=True).start()
    
    # Create a web application
    app = web.Application()
    app.add_routes([web.post('/receive_message', handle_request)])
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 5000)  # Listening on all network interfaces
    await site.start()
    print("Server is running on port 5000")

    # Keep the server running indefinitely
    while True:
        await asyncio.sleep(3600)

# If the script is run directly, initialize and run the application
if __name__ == '__main__':
    asyncio.run(init_app())
