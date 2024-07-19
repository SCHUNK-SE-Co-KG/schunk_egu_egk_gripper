from src.dummy import Dummy

from fastapi import FastAPI, Request, Form
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
from pydantic import BaseModel

# Components
dummy = Dummy()
server = FastAPI()
client = ["http://localhost:8001"]


server.add_middleware(
    CORSMiddleware,
    allow_origins=client,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=[""],
)


class Update(BaseModel):
    inst: str
    value: str
    elem: Optional[int] = None
    callback: Optional[str] = None


@server.post("/adi/update.json")
async def post(
    inst: str = Form(...),
    value: str = Form(...),
    elem: Optional[int] = Form(None),
    callback: Optional[str] = Form(None),
):
    msg = Update(inst=inst, value=value, elem=elem, callback=callback)
    return dummy.post(msg)


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
