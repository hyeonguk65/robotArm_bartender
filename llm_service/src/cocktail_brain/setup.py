from setuptools import find_packages, setup

package_name = "cocktail_brain"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools>=58.2.0",
        "google-generativeai>=0.8.3",
        "openai-whisper",
        "gtts",
        "speechrecognition",
        "pyaudio",
        "sounddevice",
        "numpy<1.25.0",
        "scipy",
    ],
    zip_safe=True,
    maintainer="lewis",
    maintainer_email="yosijki@gmail.com",
    description="LLM service for cocktail bartender robot - handles speech recognition, AI reasoning, and text-to-speech",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "brain_node = cocktail_brain.brain_node:main",
            "check_models = cocktail_brain.check_models:main",
        ],
    },
)
