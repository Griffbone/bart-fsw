#!/bin/bash

usage() { echo "Usage: $0 [-c <cubemx directory>] [-p <project directory>] [-m <mcu>]" 1>&2; exit 1; }

main() {
  while getopts ":c:p:m:e:" o; do
    case "${o}" in
      c)
        CUBEDIR=${OPTARG}
        ;;
      p)
        PROJDIR=${OPTARG}
        ;;
      m)
        MCU=${OPTARG}
        ;;
      e)
        MIDDLEWARES+=("$OPTARG")
        ;;
      *)
        usage
        ;;
    esac
  done
  shift $((OPTIND-1))

  if [ -z "${CUBEDIR}" ] || [ -z "${PROJDIR}" ] || [ -z "${MCU}" ]; then
    usage
  fi
  
  rm -rf "${PROJDIR}/src/system"
  rm -rf "${PROJDIR}/Makefile"
  rm -rf "${PROJDIR}/build"
  for val in "${MIDDLEWARES[@]}"; do
    if [ "$val" = "fatfs" ]; then
      rm -rf "${PROJDIR}/src/fatfs"
    fi
  done 
  
  mkdir -p "${PROJDIR}/src/system"
  cp "${CUBEDIR}/"startup_"${MCU,,}"*.s "${PROJDIR}/src/system/"
  cp "${CUBEDIR}/Core/Inc/"* "${PROJDIR}/src/system"
  cp "${CUBEDIR}/Core/Src/"* "${PROJDIR}/src/system"
  cp "${CUBEDIR}/"*.ld "${PROJDIR}/src/system"
  
  mv "${PROJDIR}/src/system/main.h" "${PROJDIR}/src/system/init.h"
  mv "${PROJDIR}/src/system/main.c" "${PROJDIR}/src/system/init.c"
  
  mkdir -p "${PROJDIR}/external/drivers/${MCU,,}xx/src"
  cp -r "${CUBEDIR}/Drivers/${MCU^^}xx_HAL_Driver/Src/"* \
    "${PROJDIR}/external/drivers/${MCU,,}xx/src/"
   
  mkdir -p "${PROJDIR}/external/drivers/${MCU,,}xx/include"
  cp -r "${CUBEDIR}/Drivers/${MCU^^}xx_HAL_Driver/Inc/"* \
    "${PROJDIR}/external/drivers/${MCU,,}xx/include/"
   
  mkdir -p "${PROJDIR}/external/drivers/cmsis/include"
  cp -r "${CUBEDIR}/Drivers/CMSIS/Device/ST/${MCU^^}xx/Include/"* \
    "${PROJDIR}/external/drivers/cmsis/include/"
  
  cp -r "${CUBEDIR}/Drivers/CMSIS/Include/"* \
    "${PROJDIR}/external/drivers/cmsis/include/"


  cp "${CUBEDIR}/Makefile" "${PROJDIR}/Makefile"
  
  sed -i "s/Core\/Src/src\/system/g" ${PROJDIR}/Makefile
  sed -i \
    "s/Drivers\/${MCU^^}xx_HAL_Driver\/Src/external\/drivers\/${MCU,,}xx\/src/g" \
    ${PROJDIR}/Makefile
  sed -i \
    "s/Drivers\/${MCU^^}xx_HAL_Driver\/Inc/external\/drivers\/${MCU,,}xx\/include/g" \
    ${PROJDIR}/Makefile
  sed -i \
    "s/Drivers\/CMSIS\/Device\/ST\/${MCU^^}xx\/Include/external\/drivers\/cmsis\/include/g" \
    ${PROJDIR}/Makefile
  sed -i "s/Drivers\/CMSIS\/Include/external\/drivers\/cmsis\/include/g" \
    ${PROJDIR}/Makefile
  sed -i "/Core\/Inc/d" ${PROJDIR}/Makefile
  
  LDSCRIPT=$(awk '/LDSCRIPT = /{print $NF}' ${PROJDIR}/Makefile)

  sed -i "s/LDSCRIPT = .*/LDSCRIPT = src\/system\/$(echo $LDSCRIPT | sed -e 's/[]\/$*.^[]/\\&/g')/" \
    "${PROJDIR}/Makefile"
  
  sed -i "/C_INCLUDES = /a \-Isrc \\\\" "${PROJDIR}/Makefile"
  sed -i "/C_INCLUDES = /a \-Isrc/system \\\\" "${PROJDIR}/Makefile"
  
  if ! grep -q src/system/startup.*.s ${PROJDIR}/Makefile; then
    sed -i "s/\(startup.*.s\)/src\/system\/\1/g" ${PROJDIR}/Makefile
  fi
  
  readarray -d '' CSOURCES < <(find ${PROJDIR}/src -name "*.c" -not -path \
    ${PROJDIR}"/src/system/*")
  
  for t in ${CSOURCES[@]}; do
    DIRECTORY=$(echo $t | sed "s/.*src/src/g")
    sed -i "/C_SOURCES = /a ${DIRECTORY} \\\\" "${PROJDIR}/Makefile"
  done
  
  sed -i "/src\/system\/main\.c/d" "${PROJDIR}/Makefile"
  sed -i "/C_SOURCES = /a src\/system\/init\.c \\\\" "${PROJDIR}/Makefile"
 
  BUILD_DIR=""
  if [[ "${CUBEDIR: -1:1}" == *\/* ]]; then
    BUILD_DIR=$(echo ${CUBEDIR::-1} | sed "s/.*\///g")
  else
    BUILD_DIR=$(echo ${CUBEDIR} | sed "s/.*\///g")
  fi

  sed -i "s/BUILD_DIR = build/BUILD_DIR = build\/${BUILD_DIR}/" \
    "${PROJDIR}/Makefile"

  sed -i "s/mkdir/mkdir \-p/" "${PROJDIR}/Makefile"

  sed -i "s///g" "${PROJDIR}/Makefile"

  for val in "${MIDDLEWARES[@]}"; do
    if [ "$val" = "fatfs" ]; then
      mkdir -p "${PROJDIR}/src/fatfs/app/"
      mkdir -p "${PROJDIR}/src/fatfs/target/"
      mkdir -p "${PROJDIR}/src/fatfs/src/"
      cp -r "${CUBEDIR}/FATFS/App/"* "${PROJDIR}/src/fatfs/app"
      cp -r "${CUBEDIR}/FATFS/Target/"* "${PROJDIR}/src/fatfs/target"
      cp -r "${CUBEDIR}/Middlewares/Third_Party/FatFs/src/"* \
        "${PROJDIR}/src/fatfs/src"
      sed -i "s/FATFS\/Target/src\/fatfs\/target/" "${PROJDIR}/Makefile"
      sed -i "s/FATFS\/App/src\/fatfs\/app/" "${PROJDIR}/Makefile"
      sed -i "s/Middlewares\/Third_Party\/FatFs\/src/src\/fatfs\/src/" \
        "${PROJDIR}/Makefile"
      sed -i "/C_SOURCES = /a src\/fatfs\/target\/user_diskio_spi.c \\\\" "${PROJDIR}/Makefile"
    elif [ "$val" = "dsp" ]; then
      sed -i "/C_INCLUDES = /a -Isrc\/dsp \\\\" "${PROJDIR}/Makefile"
      # sed -i "/^LIBS/ s/$/-larm_cortexM7lfdp_math/" "${PROJDIR}/Makefile"
      # sed -i "/^LIBDIR/ s/$/-Lsrc\/dsp/" "${PROJDIR}/Makefile"
      sed -i "/C_DEFS = /a -DARM_MATH_CM7 \\\\" "${PROJDIR}/Makefile"
    fi
  done


  readarray -d '' TARGETS < <(grep --null -HRl \
    --exclude-dir={"data","tools",".git"} "main\.h"  ${PROJDIR})

  for t in ${TARGETS[@]}; do
    sed -i "s/#include \"main\.h\"/#include \"init\.h\"/g" $t
  done

  sed -i "/\/\* USER CODE BEGIN EFP \*\//a int init(void)\;" \
    "${PROJDIR}/src/system/init.h"

  sed -i "s/int main(void)/int init(void)/g" "${PROJDIR}/src/system/init.c"

  sed -i "0,/\/\* USER CODE END WHILE \*\//{s/while (1)/return 0\;/}" \
    "${PROJDIR}/src/system/init.c"

}

main "${@}"
