from src.dummy import Dummy

from fastapi import FastAPI
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
    dummy.process(msg.message)
    return True


@server.get("/adi/info.json")
async def get_info():
    return dummy.get_info()


@server.get("/adi/enum.json")
async def get_enum():
    return dummy.get_enum()


@server.get("/adi/data.json")
async def get_data():
    return dummy.get_data()
