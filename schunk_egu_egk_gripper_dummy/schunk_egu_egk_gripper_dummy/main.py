from src.dummy import Dummy

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
from pydantic import BaseModel

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


class Message(BaseModel):
    message: str
    optional: Optional[str] = None


@server.put("/")
async def put(msg: Message):
    print(msg)
    return True


@server.get("/adi/{path}")
async def get(request: Request):
    path = request.path_params["path"]
    params = request.query_params
    if path == "info.json":
        return dummy.get_info(params)
    if path == "enum.json":
        return dummy.get_enum(params)
    if path == "data.json":
        return dummy.get_data(params)
    return None
