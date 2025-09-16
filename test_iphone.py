import threading, queue, requests, time

url = "http://192.168.0.119"
q = queue.Queue()

def fetch():
    while True:
        try:
            r = requests.get(url, timeout=2)
            q.put(r.content)
        except:
            q.put(None)

threading.Thread(target=fetch, daemon=True).start()

frames = []
start = time.time()
for _ in range(10):
    while q.empty():
        time.sleep(0.001)
    frames.append(q.get())
end = time.time()

print(f"FPS threaded: {len(frames)/(end-start):.2f}")
