from setuptools import setup,find_packages

setup(
    name="obstacle_detection",
    version="1.0.0",
    description="A package for Hakuroukun",
    url="https://ise.tut.ac.jp/",
    author="Dinh Ngoc Duc",
    author_email="dinh.ngoc.duc.mo@tut.ac.jp",
    license="MIT",
    packages=find_packages("obstacle_detection"),
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "License :: OSI Approved :: BSD License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
    ],
)
