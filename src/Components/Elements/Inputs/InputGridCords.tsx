import React, { useEffect, useState } from "react";
import TextField, { TextFieldProps } from "@mui/material/TextField";
import { SxProps, Theme } from "@mui/material/styles";

type CompactsInputProps = TextFieldProps & {
  customSx?: SxProps<Theme>; // opcional, si querés estilos extra personalizados  
  value: number;
  onChange: (e: React.ChangeEvent<HTMLInputElement>) => void; 
  //onChange: (value: number) => void; 

};

const InputGridCords: React.FC<CompactsInputProps> = ({value, onChange, sx, ...props }) => {

  const [displayVal, setDisplayVal] = useState(value.toFixed(2));
  
    // 🔄 Sincroniza el valor visible si el valor real cambia externamente
    useEffect(() => {
      setDisplayVal(value.toFixed(2));
    }, [value]);
  
    const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
      let val = e.target.value;
  
      // Permitir negativos y decimales
      val = val.replace(/[^0-9.-]/g, "");
      if (val.includes("-")) val = val.replace(/(?!^)-/g, "");
      const parts = val.split(".");
      if (parts.length > 2) val = parts[0] + "." + parts.slice(1).join("");
  
      setDisplayVal(val); // 🔹 Solo cambia lo que se ve, no el valor real todavía
    };
  
    const handleFocus = (e: React.FocusEvent<HTMLInputElement>) => {
      e.target.select(); // Selecciona todo el texto al enfocar
    };
  
    const handleBlur = (e: React.FocusEvent<HTMLInputElement>) => {
      let num = parseFloat(displayVal);
      if (isNaN(num)) num = value; // si queda vacío, mantiene el anterior

      // 🔹 Creamos un evento sintético compatible con el padre
      const syntheticEvent = {
        ...e,
        target: { ...e.target, value: num.toString() },
      } as unknown as React.ChangeEvent<HTMLInputElement>;

      onChange(syntheticEvent); // Llamamos al padre con un evento
      setDisplayVal(num.toFixed(2));
    }
    
  return (
    <TextField
      size="small"
      type="number"
      {...props}


      value={displayVal}
      onChange={handleChange}
      onBlur={handleBlur}
      onFocus={handleFocus}
      inputProps={{
        inputMode: "decimal",
        pattern: "-?[0-9]*[.]?[0-9]*",
        inputProps: {
          step: '1',
          inputMode: 'numeric',
        },
      }}


      //InputLabelProps={{ shrink: true }}
      sx={{
        width: 55,
        height: '100%',
        /*
        '& input::-webkit-outer-spin-button': {
          display: 'none',
        },
        '& input::-webkit-inner-spin-button': {
          display: 'none',
        },*/
        '& .MuiInputBase-root': {
          fontSize: '0.7rem',
          padding: '0px 4px',
          minHeight: '2px',
        },
        '& .MuiInputLabel-root': {
          fontSize: '0.70rem',
        },
        '& input': {
          padding: 0,
        },
      }}
    />
  );
};

export default InputGridCords;
