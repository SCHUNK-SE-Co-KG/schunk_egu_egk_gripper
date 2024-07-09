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


@server.get("/")
async def get():
    return "Dummy ready"
