#!/usr/bin/env python3


def setup():
    pass


def loop():
    pass


if __name__ == "__main__":
    error_msg = (
        "Você não deve executar o arquivo control.py!\n"
        + "Para iniciar o projeto, use o comando rosrun <nome_do_projeto> run.py"
    )
    raise Exception(error_msg)
