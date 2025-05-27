from schunk_gripper_dummy.dummy import Dummy

from fastapi import FastAPI, Request, Form, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import string

# Components
dummy = Dummy()
dummy.start()
server = FastAPI()
client = ["http://localhost:8001"]


server.add_middleware(
    CORSMiddleware,
    allow_origins=client,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=[""],
)


@server.post("/adi/update.json")
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


@server.get("/adi/{path}")
async def get(request: Request):
    path = request.path_params["path"]
    params = dict(request.query_params)
    if path == "info.json":
        return dummy.get_info(params)
    if path == "enum.json":
        return dummy.get_enum(params)
    if path == "data.json":
        return dummy.get_data(params)
    return None
