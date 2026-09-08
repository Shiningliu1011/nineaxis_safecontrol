from setuptools import setup, find_packages

setup(
    name='dpax',
    version='0.0.1',
    packages=find_packages(),
    install_requires=['jax', 'jaxlib'],
    python_requires='>=3.8',
    description='DCOL Differentiable Collision Detection (JAX port by Kevin Tracy)',
    url='https://github.com/kevin-tracy/dpax',
)
