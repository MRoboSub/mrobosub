import pandas as pd
import numpy as np
import yaml

PWM_COL_NAME = 'PWM(µs)'
FORCE_COL_KEY = 'Force(Kgf)'

PWM_ZERO = 1500
PWM_DEADBAND = 27

EXCEL_FILE_NAME = "T200-Public-Performance-Data-10-20V-September-2019.xlsx"

def get_fit(data):
    x: np.ndarray = data[PWM_COL_NAME].to_numpy()
    y: np.ndarray = data[FORCE_COL_KEY].to_numpy()
    fit: np.ndarray = np.polyfit(x, y, 2)
    def index_to_letter(index):
        return chr(ord('a') + index)
    return dict([(index_to_letter(i), val) for (i,val) in enumerate(fit.tolist())])

def main():
    sheets = pd.read_excel(EXCEL_FILE_NAME, sheet_name=None)

    output = {
        'fits': []
    }

    for (name, sheet) in sheets.items():
        if not 'V' in name:
            continue

        voltage = float(name[0:2])

        # Remove spaes from column names
        sheet.columns = sheet.columns.str.replace(' ', '')

        fwd_data: pd.DataFrame = sheet[sheet[PWM_COL_NAME] > PWM_ZERO + PWM_DEADBAND]
        rev_data: pd.DataFrame = sheet[sheet[PWM_COL_NAME] < PWM_ZERO - PWM_DEADBAND]

        profile = {
            'voltage': voltage,
            'fwd': get_fit(fwd_data),
            'rev': get_fit(rev_data),
        }

        output['fits'].append(profile)

    with open('fits.yaml', 'w') as f:
        f.write(yaml.dump(output, sort_keys=False))

if __name__ == '__main__':
    main()
