#!/bin/bash

usage()
{
	echo "Usage: $program_name <init|help> [params...]"
	echo "   help - Display this help text"
	echo "   init"
	echo "      - Initialize an Ament build tree"
	echo "   create <component>"
	echo "      - Create a component of a project"
}

program_name=$0
sub_command=$1
script_dir="$( dirname "${BASH_SOURCE[0]}" )"

create_usage()
{
	echo ""
	echo "Usage: $program_name create <component> [args...]"
	echo "   help"
	echo "      - Display this help text"
	echo "   package <path> <python|cpp>"
	echo "      - Initialize a Python or C++ package at the specified path"
	echo "      <path> - Path to the package in the ./src directory (not including src)"
	echo "   file   <package-path> <name> [template]"
	echo "   header <package-path> <name> [template]"
	echo "      - Create a code file with an optional template"
	echo "      <package-path> - Path to the package in the ./src directory (not including src)"
	echo "      <name> - Name of file to create in the package's source directory"
	echo "      [template] - Template from $script_dir/templates to copy"
	echo ""
}

parse_create()
{
	if [ -z $2 ]; then
		echo "Create requires a package path!"
		create_usage
		exit 1
	fi
	package_dir="src/$2"

	case "$1" in
		"package" )
			if [ -z $3 ]; then
				echo "You must specify language!"
				create_usage
				exit 1
			fi

			mkdir -p "$package_dir"

			read -p "   Description: " package_desc
			read -p "   License [Apache License 2.0]: " package_license
			package_license=${package_license:-"Apache License 2.0"}
			read -p "   Maintainer: " package_maintainer
			read -p "   Maintainer Email: " package_maintainer_email

			case "$3" in
				"python" )
					init_python "$package_dir" "$package_desc" "$package_license" "$package_maintainer" "$package_maintainer_email"
					;;
				"cpp" )
					init_cpp "$package_dir" "$package_desc" "$package_license" "$package_maintainer" "$package_maintainer_email"
					;;
				* )
					create_usage
					;;
			esac
			;;
		"file" )
			if [ -z $3 ]; then
				echo "Create needs a file name!"
				create_usage
				exit 1
			fi

			if [ -e "$package_dir/setup.py" ]; then
				# Package is Python
				package_name=$(basename $package_dir)
				src_dir="$package_dir/$package_name"
			else
				# Package is C++
				src_dir="$package_dir/src"
			fi
			
			comp_file="$src_dir/$3"
			mkdir -p $( dirname $comp_file )

			[ -n $4 ] && cp "$script_dir/templates/$4" "$comp_file" || touch $comp_file
			;;
		"header" )
			if [ -z $3 ]; then
				echo "Create needs a file name!"
				create_usage
				exit 1
			fi

			inc_dir="$package_dir/include"
			
			comp_file="$inc_dir/$3"
			mkdir -p $( dirname $comp_file )

			[ -n $4 ] && cp "$script_dir/templates/$4" "$comp_file" || touch $comp_file
			;;
		* )
			create_usage
			;;
	esac
}



init_usage()
{
	echo "Usage: $program_name init <project|package|help> [params...]"
	echo "   help - Display this help text"
	echo "   project [company]"
	echo "      - Initialize the project structure, optionally adding a company folder in ./src/"
}
parse_init()
{
	init_command=$1
	case $init_command in
		"project" )
			mkdir -p "src/$2"
			;;
		"package" )
			if [ -z $2 ]; then
				echo "Package requires a name!"
				init_usage
				exit 1
			fi
			if [ -z $3 ]; then
				echo "You must specify language!"
				init_usage
				exit 1
			fi
			package_dir="src/$2"
			mkdir -p "$package_dir"

			read -p "   Description: " package_desc
			read -p "   License [Apache License 2.0]: " package_license
			package_license=${package_license:-"Apache License 2.0"}
			read -p "   Maintainer: " package_maintainer
			read -p "   Maintainer Email: " package_maintainer_email

			case "$3" in
				"python" )
					init_python "$package_dir" "$package_desc" "$package_license" "$package_maintainer" "$package_maintainer_email"
					;;
				"cpp" )
					init_cpp "$package_dir" "$package_desc" "$package_license" "$package_maintainer" "$package_maintainer_email"
					;;
				* )
					init_usage
					;;
			esac
			;;
		* )
			init_usage
			;;
	esac
}
init_package_xml()
{
	package_file="$1/package.xml"
	package_name=$(basename $1)
	language=$2
	package_desc=$3
	package_license=$4
	package_maintainer=$5
	package_maintainer_email=$6

	echo '<?xml version="1.0"?>' > $package_file
	echo '<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>' >> $package_file
	echo '<package format="3">' >> $package_file
	echo "  <name>$package_name</name>" >> $package_file
	echo '  <version>0.0.1</version>' >> $package_file
	echo "  <description>$package_desc</description>" >> $package_file
	[ -n "$package_maintainer" ] && ( [ -n $package_maintainer_email ] \
		&& echo "  <maintainer email=\"$package_maintainer_email\">$package_maintainer</maintainer>" >> $package_file \
		|| echo "  <maintainer>$package_maintainer</maintainer>" >> $package_file \
	)
	echo "  <license>${package_license}</license>" >> $package_file
	echo -e "\n" >> $package_file
	if [ "$language" = "cpp" ]; then
		echo -e "  <buildtool_depend>ament_cmake</buildtool_depend>\n" >> $package_file
		echo -e "  <build_depend>rclcpp</build_depend>" >> $package_file
		echo -e "  <build_depend>rcutils</build_depend>\n" >> $package_file
		echo -e "  <exec_depend>rclcpp</exec_depend>" >> $package_file
		echo -e "  <exec_depend>rcutils</exec_depend>\n" >> $package_file
		echo -e "  <export>\n    <build_type>ament_cmake</build_type>\n  </export>\n</package>" >> $package_file
	elif [ "$language" = "python" ]; then
		echo -e "  <exec_depend>rclpy</exec_depend>\n" >> $package_file
		echo -e "  <export>\n    <build_type>ament_python</build_type>\n  </export>\n</package>" >> $package_file
	fi
}
init_python()
{
	init_package_xml "$1" "python" "$package_desc" "$package_license" "$package_maintainer" "$package_maintainer_email"
	setup_file="$1/setup.py"
	package_name=$(basename $1)
	[ -d "$1/$package_name" ] || mkdir "$1/$package_name"

	package_desc=$2
	package_license=$3
	package_maintainer=$4
	package_maintainer_email=$5
	read -p "   Author [${package_maintainer:-"none"}]: " package_author
		package_author=${package_author:-$package_maintainer}
	if [ -n $package_author ]; then
		read -p "   Author Email [${package_maintainer_email:-"none"}]: " package_author_email
		package_author_email=${package_author_email:-$package_maintainer_email}
	fi

	echo -e "from setuptools import find_packages" > $setup_file
	echo -e "from setuptools import setup\n" >> $setup_file
	echo -e "package_name = '$package_name'\n" >> $setup_file
	echo -e "setup(" >> $setup_file
	echo -e "    name=package_name," >> $setup_file
	echo -e "    version='0.0.1'," >> $setup_file
	echo -e "    packages=find_packages(exclude=['test'])," >> $setup_file
	echo -e "    data_files=[" >> $setup_file
	echo -e "        ('share/ament_index/resource_index/packages'," >> $setup_file
	echo -e "            ['resource/' + package_name])," >> $setup_file
	echo -e "        ('share/' + package_name, ['package.xml'])," >> $setup_file
	echo -e "    ]," >> $setup_file
	echo -e "    install_requires=['setuptools']," >> $setup_file
	echo -e "    zip_safe=True," >> $setup_file
	[ -n "$package_author" ] &&  echo -e "    author='$package_author'," >> $setup_file
	[ -n "$package_author_email" ] &&  echo -e "    author_email='$package_author_email'," >> $setup_file
	[ -n "$package_maintainer" ] &&  echo -e "    maintainer='$package_maintainer'," >> $setup_file
	[ -n "$package_maintainer_email" ] &&  echo -e "    maintainer_email='$package_maintainer_email'," >> $setup_file
	echo -e "    keywords=['ROS']," >> $setup_file
	echo -e "    classifiers=[" >> $setup_file
	echo -e "        'Intended Audience :: Developers'," >> $setup_file
	echo -e "        'Programming Language :: Python'," >> $setup_file
	echo -e "        'Topic :: Software Development'," >> $setup_file
	echo -e "    ]," >> $setup_file
	echo -e "    description=(" >> $setup_file
	echo -e "        '$package_desc'" >> $setup_file
	echo -e "    )," >> $setup_file
	echo -e "    license='$package_license'," >> $setup_file
	echo -e "    tests_require=['pytest']," >> $setup_file
	echo -e "    entry_points={" >> $setup_file
	echo -e "        'console_scripts': [" >> $setup_file
	echo -e "#            'command_name = $package_name.subfolder.file_name:main'," >> $setup_file
	echo -e "        ]," >> $setup_file
	echo -e "    }," >> $setup_file
	echo -e ")" >> $setup_file



	config_file="$1/setup.cfg"
	echo -e "[develop]" > $config_file
	echo -e "script-dir=\$base/lib/$package_name" >> $config_file
	echo -e "[install]" >> $config_file
	echo -e "install-scripts=\$base/lib/$package_name" >> $config_file



	[ -d "$1/resource" ] || mkdir "$1/resource"
	touch "$1/resource/$package_name"
}
init_cpp()
{
	init_package_xml "$1" "cpp" "$package_desc" "$package_license" "$package_maintainer" "$package_maintainer_email"
	cmake_file="$1/CMakeLists.txt"
	package_name=$(basename $1)
	[ -d "$1/src" ] || mkdir "$1/src"

	echo -e "cmake_minimum_required(VERSION 3.5)\n" > $cmake_file
	echo -e "set(PROJECT_NAME $package_name)" >> $cmake_file
	echo -e "project(\${PROJECT_NAME})\n" >> $cmake_file
	echo -e "# Default to C++14" >> $cmake_file
	echo -e "if(NOT CMAKE_CXX_STANDARD)" >> $cmake_file
	echo -e "  set(CMAKE_CXX_STANDARD 14)" >> $cmake_file
	echo -e "endif()\n" >> $cmake_file
	echo -e "if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")" >> $cmake_file
	echo -e "  set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic\")" >> $cmake_file
	echo -e "endif()\n" >> $cmake_file
	echo -e "find_package(ament_cmake REQUIRED)" >> $cmake_file
	echo -e "find_package(rclcpp REQUIRED)" >> $cmake_file
	echo -e "find_package(rcutils)\n" >> $cmake_file
	echo -e "function(custom_executable subfolder target)" >> $cmake_file
	echo -e "  add_executable(\${target} src/\${subfolder}/\${target}.cpp)" >> $cmake_file
	echo -e "  ament_target_dependencies(\${target}" >> $cmake_file
	echo -e "    \"rclcpp\"" >> $cmake_file
	echo -e "    \"rcutils\")" >> $cmake_file
	echo -e "  install(TARGETS \${target}" >> $cmake_file
	echo -e "  DESTINATION lib/\${PROJECT_NAME})" >> $cmake_file
	echo -e "endfunction()\n\n" >> $cmake_file
	echo -e "# custom_executable(subfolder filename)\n\n" >> $cmake_file
	echo -e "ament_package()" >> $cmake_file
}





case $sub_command in
	"init" )
		parse_init "${@:2}"
		;;
	"create" )
		parse_create "${@:2}"
		;;
	* )
		usage
		;;
esac