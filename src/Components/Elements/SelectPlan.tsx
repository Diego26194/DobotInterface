import React from "react";
import { Select, MenuItem, SelectProps } from "@mui/material";
import { SxProps, Theme } from "@mui/material/styles";

type Select1Props = SelectProps & {
  customSx?: SxProps<Theme>;
};

const SelectPlan: React.FC<Select1Props> = ({ sx, value, onChange , ...props}) => {
  const opcionesTrayectoria = ['PTP', 'LIN', 'CIRC'];
  return (
    <Select
      size="small"
      displayEmpty
      value={value}        
      onChange={onChange}
      sx={{
        width: "100%",
        fontSize: "0.75rem",
        padding: "0px 2px",
        minHeight: "16px",
        maxHeight: "24px",
        textAlign: "center",
        ...sx,
      }}
      {...props}
    >
      {opcionesTrayectoria.map((opcion) => (
              <MenuItem 
                key={opcion} 
                value={opcion} 
                sx={{
                  width: "100%",
                  fontSize: "0.75rem",
                  padding: "0px 12px",
                  minHeight: "24px",
                  textTransform: "none",
                  justifyContent: "center",
                }}>
                {opcion}
              </MenuItem>
      ))}
    </Select>
  );
};

export default SelectPlan;