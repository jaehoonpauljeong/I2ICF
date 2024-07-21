import asyncio
from aiohttp import web

async def handle_request_9090(request):
    message = await request.text()
    print("Received message from port 9090:")
    print(message)
    
    # 메시지를 하나의 파일에 저장, 포트 정보 포함
    with open('SDV_UE_messages.txt', 'a') as file:
        file.write(f"Port 9090: {message}\n")
    
    return web.Response(text="Message received on port 9090", status=200)

async def handle_request_9091(request):
    message = await request.text()
    print("Received message from port 9091:")
    print(message)
    
    # 메시지를 하나의 파일에 저장, 포트 정보 포함
    with open('SDV_UE_messages.txt', 'a') as file:
        file.write(f"Port 9091: {message}\n")
    
    return web.Response(text="Message received on port 9091", status=200)

async def init_app():
    app_9090 = web.Application()
    app_9091 = web.Application()
    
    app_9090.add_routes([web.post('/receive_message', handle_request_9090)])
    app_9091.add_routes([web.post('/receive_message', handle_request_9091)])
    
    runner_9090 = web.AppRunner(app_9090)
    runner_9091 = web.AppRunner(app_9091)
    
    await runner_9090.setup()
    await runner_9091.setup()
    
    site_9090 = web.TCPSite(runner_9090, '10.60.0.4', 9090)
    site_9091 = web.TCPSite(runner_9091, '10.60.0.4', 9091)
    
    await site_9090.start()
    await site_9091.start()
    
    print("Servers are running on ports 9090 and 9091")
    while True:
        await asyncio.sleep(3600)

if __name__ == '__main__':
    asyncio.run(init_app())
