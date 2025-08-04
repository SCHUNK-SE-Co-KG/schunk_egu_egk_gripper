import argparse
import uvicorn
from schunk_gripper_dummy.dummy import Dummy
from fastapi import FastAPI, Request, Form, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import string


def create_webserver(dummy: Dummy):
    webserver = FastAPI()
    clients: list[str] = []

    webserver.add_middleware(
        CORSMiddleware,
        allow_origins=clients,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @webserver.post("/adi/update.json")
    async def post(
        inst: str = Form(...),
        value: str = Form(...),
        elem: Optional[int] = Form(None),
        callback: Optional[str] = Form(None),
        background_tasks: BackgroundTasks = BackgroundTasks(),
    ):
        msg = {"inst": inst, "value": value, "elem": elem, "callback": callback}
        if msg["inst"] not in dummy.data:
            return {"result": 1}
        if not all(digit in string.hexdigits for digit in str(msg["value"])):
            return {"result": 1}
        background_tasks.add_task(dummy.post, msg)
        return {"result": 0}

    @webserver.get("/adi/{path}")
    async def get(request: Request):
        path = request.path_params["path"]
        params = dict(request.query_params)
        if path == "data.json":
            return dummy.get_data(params)
        return None

    return webserver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    dummy = Dummy()
    dummy.start()
    webserver = create_webserver(dummy)
    try:
        uvicorn.run(webserver, host="0.0.0.0", port=args.port)
    except KeyboardInterrupt:
        dummy.stop()
        print("Server stopped by user.")


if __name__ == "__main__":
    main()
