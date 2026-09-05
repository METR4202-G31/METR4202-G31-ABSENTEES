from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'test_worlds'
package_root = os.path.dirname(os.path.abspath(__file__))

model_data_files = []
for model_dir in glob(os.path.join(package_root, 'models', '*')):
    if os.path.isdir(model_dir):
        model_name = os.path.basename(model_dir)
        for source_dir, _, filenames in os.walk(model_dir):
            relative_dir = os.path.relpath(source_dir, package_root)
            model_data_files.append((
                os.path.join('share', package_name, relative_dir),
                [os.path.join(relative_dir, filename)
                 for filename in filenames],
            ))

resource_data_files = [
    ('share/' + package_name + '/resource/media/materials/scripts',
     [os.path.relpath(path, package_root) for path in glob(
         os.path.join(package_root, 'resource/media/materials/scripts/*'))]),
    ('share/' + package_name + '/resource/media/materials/textures',
     [os.path.relpath(path, package_root) for path in glob(
         os.path.join(package_root, 'resource/media/materials/textures/*'))]),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        *model_data_files,
        *resource_data_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zak',
    maintainer_email='zak@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'generate_world = test_worlds.world_generator.main:main',
        ],
    },
)
