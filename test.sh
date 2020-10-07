git secrets --scan 2>&1 >/dev/null | grep "[ERROR]"
if [ $? == 0 ] 
then 
    _secret_exists=1
    echo "secrets exist in your commits. please rectify and re-commit."
    exit 1
else
    _secret_exists=0
    exit 0
fi 
