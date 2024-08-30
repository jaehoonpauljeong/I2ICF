import asyncio
from aiohttp import web, ClientSession

# CloudServer IP
CloudServer_IP = '10.60.0.10'
# SDVUser IP to forward messages
SDVUser_IP = '10.60.0.11'
SDVUser_port = 5000

# Define an asynchronous function to handle requests on port 9090
async def handle_request_9090(request):
    message = await request.text()
    print("Received message from port 9090:")
    print(message)
    
    # Append the message to a file, including port information
    with open('SDV_UE_messages.txt', 'a') as file:
        file.write(f"Port 9090: {message}\n")
    
    # Forward the message to SDVuser
    await forward_message(message)
    
    return web.Response(text="Message received and forwarded from port 9090", status=200)

# Define an asynchronous function to handle requests on port 9091
async def handle_request_9091(request):
    message = await request.text()
    print("Received message from port 9091:")
    print(message)
    
    # Append the message to a file, including port information
    with open('SDV_UE_messages.txt', 'a') as file:
        file.write(f"Port 9091: {message}\n")
    
    # Forward the message to SDVuser
    await forward_message(message)
    
    return web.Response(text="Message received and forwarded from port 9091", status=200)

# Define an asynchronous function to forward messages to the new server
async def forward_message(message):
    async with ClientSession() as session:
        try:
            # Send the message to the new server
            async with session.post(f'http://{SDVUser_IP}:{SDVUser_port}/receive_message', data=message) as response:
                # Print the status code and response text from the new server
                print(f"Forwarded to {SDVUser_IP}: {response.status}, {await response.text()}")
        except Exception as e:
            print(f"Failed to forward message: {e}")


# Define an asynchronous function to initialize the web applications
async def init_app():
    app_9090 = web.Application()
    app_9091 = web.Application()
    
    app_9090.add_routes([web.post('/receive_message', handle_request_9090)])
    app_9091.add_routes([web.post('/receive_message', handle_request_9091)])
    
    runner_9090 = web.AppRunner(app_9090)
    runner_9091 = web.AppRunner(app_9091)
    
    await runner_9090.setup()
    await runner_9091.setup()
    
    site_9090 = web.TCPSite(runner_9090, CloudServer_IP, 9090)
    site_9091 = web.TCPSite(runner_9091, CloudServer_IP, 9091)
    
    await site_9090.start()
    await site_9091.start()

    print("Servers are running on ports 9090 and 9091")
    
    while True:
        await asyncio.sleep(3600)

if __name__ == '__main__':
    asyncio.run(init_app())

