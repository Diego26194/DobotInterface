import React from "react";
import TextField, { TextFieldProps } from "@mui/material/TextField";
import { SxProps, Theme } from "@mui/material/styles";

type CompactsInputProps = TextFieldProps & {
  customSx?: SxProps<Theme>; // opcional, si querés estilos extra personalizados
};

const CompactsInput: React.FC<CompactsInputProps> = ({ sx, ...props }) => {
  return (
    <TextField
      size="small"
      fullWidth
      {...props}
      //InputLabelProps={{ shrink: true }}
      sx={{
        height: '100%',
        '& .MuiInputBase-root': {
          fontSize: '0.9rem',
          padding: '0px 6px',
          minHeight: '15px',
        },
        '& .MuiInputLabel-root': {
          fontSize: '0.75rem',
        },
        '& input': {
          padding: 0,
        },
        ...(sx || {}),
      }}
    />
  );
};

export default CompactsInput;
