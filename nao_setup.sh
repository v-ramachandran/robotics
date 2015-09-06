alias nao_tool='$NAO_HOME/bin/tool'
alias mynao='ssh nao@11.0.1.47'
alias copy_py='~/nao/trunk/build/copy_robot 11.0.1.47 python'
alias pingnao='ping 11.0.1.47'
function deploy {
  cd ~/nao/trunk/build
  ./compile everything
  ./copy_robot 11.0.1.47 everything
}
