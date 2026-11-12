"""Shared station flag registration, including the temporary legacy spelling."""
import argparse


def add_station_argument(parser, *, alias, dest, help):
    parser.add_argument("--station", dest=dest, metavar="PATH", default=None, help=help)
    parser.add_argument(
        alias, dest=dest, metavar="PATH", default=argparse.SUPPRESS,
        help="--station (alias, will be removed)",
    )
