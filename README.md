# Flying Car ND Project 0

TODO: Project description here

## Getting Started

Make sure you have [Anaconda](https://www.anaconda.com/download/) installed, this is required to create the project environment.

Creating the environment:

```sh
conda env create -f environment.yml
```

If the environment was installed succesfully you should output similar to the following:

```sh
Installing collected packages: future, lxml, pymavlink, utm
Successfully installed future-0.16.0 lxml-4.0.0 pymavlink-2.2.4 utm-0.4.2
#
# To activate this environment, use:
# > source activate flyingcarnd-p0
#
# To deactivate an active environment, use:
# > source deactivate
#
```

You can view the environments you have installed with:

```sh
$ conda env list

# Sample output
# conda environments:
#
flyingcarnd-p0           /usr/local/anaconda3/envs/flyingcarnd-p0
root                  *  /usr/local/anaconda3
```

In order to use the environment you must activate it. Activating the environment:

```sh
source activate flyingcarnd-p0
```

Deactivating the environment:

```sh
source deactivate
```

Once you've installed the environment you can cleanup unused packages and tarballs:

```sh
conda clean -tpy
```

Removing the environment:

```sh
conda env remove -n flyingcarnd-p0
```
