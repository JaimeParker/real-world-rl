"""Shared station flag registration."""


def add_station_argument(parser, *, dest, help):
    parser.add_argument("--station", dest=dest, metavar="PATH", default=None, help=help)
