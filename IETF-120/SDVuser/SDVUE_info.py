import asyncio
from aiohttp import web

# Define an asynchronous function to handle incoming requests
async def handle_request(request):
    # Read the text from the incoming request
    message = await request.text()
    # Print the received message
    print("Received message:")
    print(message)
    
    # Return a response indicating that the message was received
    return web.Response(text="Message received", status=200)

# Define an asynchronous function to initialize the web application
async def init_app():
    # Create a web application
    app = web.Application()
    
    # Add routes to the web application, linking the '/receive_message' path to the handle_request function
    app.add_routes([web.post('/receive_message', handle_request)])
    
    # Create a runner to manage the web application
    runner = web.AppRunner(app)
    
    # Setup the runner
    await runner.setup()
    
    # Create a site to bind the application to the specified IP address and port
    site = web.TCPSite(runner, '0.0.0.0', 5000)  # Use '0.0.0.0' to listen on all network interfaces
    
    # Start the site
    await site.start()

    # Print a message indicating that the server is running
    print("Server is running on port 5000")
    
    # Keep the server running indefinitely
    while True:
        await asyncio.sleep(3600)

# If the script is run directly, initialize and run the application
if __name__ == '__main__':
    asyncio.run(init_app())


