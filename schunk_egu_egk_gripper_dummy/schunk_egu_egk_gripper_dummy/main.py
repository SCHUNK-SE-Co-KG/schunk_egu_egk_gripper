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
    return dummy.get(request.path_params["path"], request.query_params)
