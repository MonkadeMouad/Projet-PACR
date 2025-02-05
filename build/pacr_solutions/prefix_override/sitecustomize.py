import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/profil/monkade1u/PACR/hammou/install/pacr_solutions'
