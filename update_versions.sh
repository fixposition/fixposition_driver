#!/bin/bash

version=${1:-0.0.1}
sed -i "s@<version>.*</version>@<version>${version}</version>@" fixposition_*/package.xml
