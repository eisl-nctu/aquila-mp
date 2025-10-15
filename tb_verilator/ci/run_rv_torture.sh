#!/bin/bash
#set -e
#set -x
#$fname_array
PWD=`pwd`
NUM_CPU=8
LOG_PROCCESS_PATH=../tools/log_proccess
RV_TORTURE_PATH=../tmp/riscv-torture/output/
V_MODEL=../core_obj_dir/Vaquila_testharness
LOG_DIR=./torture_log
ORIGIN_MEM=dump.mem
ORIGIN_CPU_LOG=cpu.log
ORIGIN_DEC_LOG=dec_cpu.log
ORIGIN_VCD=aquila_core.vcd
SAVE_LOG=1


# check $1 argument exists
if [ -z "$1" ]; then
  echo "no specify log mode, default save log"
elif [ "$1" = "0" ]; then
  echo "no save log mode"
  let SAVE_LOG=0
else
  echo "save log mode"
fi

RUN_LOG="$torture_run.log"

if [ "$SAVE_LOG" -eq "1" ]; then
  if [ -f $RUN_LOG ]; then
    echo "$RUN_LOG exist, change olf file name to bak"
    mv $RUN_LOG "${RUN_LOG}.bak"
  fi

  # check log_proccess exist
  if [ ! -f $LOG_PROCCESS_PATH ]; then
    echo "$LOG_PROCCESS_PATH dose not exist, build start..."
    cd ../tools
    make log_proccess
    cd ../ci
  fi
  # check log dir exist
  if [ ! -d $LOG_DIR ]; then
    echo "$LOG_DIR dose not exists, create new one"
    mkdir -p $LOG_DIR
    mkdir -p $LOG_DIR/cpu
    mkdir -p $LOG_DIR/mem
    mkdir -p $LOG_DIR/objdump
    mkdir -p $LOG_DIR/dec_log
    mkdir -p $LOG_DIR/vcd
  else
    echo "$LOG_DIR exist, backup..."
    mv $LOG_DIR "${LOG_DIR}.bak"
    mkdir -p $LOG_DIR
    mkdir -p $LOG_DIR/cpu
    mkdir -p $LOG_DIR/mem
    mkdir -p $LOG_DIR/objdump
    mkdir -p $LOG_DIR/dec_log
    mkdir -p $LOG_DIR/vcd
  fi
fi
# check riscv-tests exist
if [ ! -d $RV_TORTURE_PATH ]; then
  echo "$RV_TORTURE_PATH dose not exist, build start..."
  cd ../
  make build_riscv_torture -j${NUM_CPU}
  cd ci
fi

# check verilator model exist
if [ ! -f $V_MODEL ]; then
  echo "$V_MODEL dose not exist, build start..."
  cd ../
  make core_verilate
  cd ci
fi

cat "${RV_TORTURE_PATH}/torture.list"
readarray fname_array < ${RV_TORTURE_PATH}/torture.list

let total=0
let err_case=0

for row in "${fname_array[@]}";do
  testcase=$(echo "$row" | tr -d '\n' )
  printf "start ${testcase} ..."
  echo "================================================================" >> $RUN_LOG
  echo "testcase: ${testcase}" >> $RUN_LOG
  ./$V_MODEL $RV_TORTURE_PATH/$testcase 1 >> $RUN_LOG
  ret_code=$?
  if [ $ret_code -ne 0 ]; then
    printf " failed"
    let "err_case=err_case+1"
  fi
  printf "\n"
  let "total=total+1"
  if [ "${SAVE_LOG}" -eq "1" ] ; then
    ./${LOG_PROCCESS_PATH} "${RV_TORTURE_PATH}/${testcase}.dump" $ORIGIN_CPU_LOG
    mv $ORIGIN_DEC_LOG "${LOG_DIR}/dec_log/${testcase}.dec.log"
    mv $ORIGIN_MEM "${LOG_DIR}/mem/${testcase}.mem.log"
    mv $ORIGIN_CPU_LOG "${LOG_DIR}/cpu/${testcase}.cpu.log"
    mv $ORIGIN_VCD "${LOG_DIR}/vcd/${testcase}.vcd"
    cp "${RV_TORTURE_PATH}/${testcase}.dump" "${LOG_DIR}/objdump/${testcase}.objdump"
  else
    rm $ORIGIN_CPU_LOG
    rm $ORIGIN_MEM
    rm $ORIGIN_VCD
  fi
done

let pass_case=$total-$err_case
echo "Total $total cases, pass: $pass_case, fail: $err_case"

if [ $err_case -ne 0 ]; then
  if [ "${SAVE_LOG}" -ne "1" ]; then
    cat ${RUN_LOG}
  fi
  exit 1
else
  exit 0
fi
