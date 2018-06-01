#!/bin/bash

chronyc -a 'burst 4/4'
chronyc -a makestep
