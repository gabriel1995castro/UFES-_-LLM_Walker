from setuptools import find_packages, setup

package_name = 'teste_walker_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabriel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'receptNode = teste_walker_llm.recept_node:main',
            'controlNode = teste_walker_llm.Control_node:main',
            'service_node = teste_walker_llm.service_node:main',
            'ollama_node = teste_walker_llm.ollama_node:main',
        ],
    },
)
