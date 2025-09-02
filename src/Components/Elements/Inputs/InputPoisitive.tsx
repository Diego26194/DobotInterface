import React from "react";
import { styled } from "@mui/material/styles";
import TextField from "@mui/material/TextField";

const NoSpinnerInput = styled(TextField)(({ theme }) => ({
  "& input::-webkit-outer-spin-button, & input::-webkit-inner-spin-button": {
    WebkitAppearance: "none",
    margin: 0,
  },
  "& input[type=number]": {
    MozAppearance: "textfield", // Firefox
  },
}));

interface InputPositiveProps {
  valMin: number;
  valMax: number;
  value: number;
  onChange: (val: number) => void;
  label?: string;
  variant?: "outlined" | "filled" | "standard";
  sx?: object;
}

const InputPositive: React.FC<InputPositiveProps> = ({
  value,
  onChange,
  label,
  variant = "outlined",
  valMin,
  valMax,
  sx,
}) => {
  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    let val = e.target.value;

    // Eliminar cualquier caracter que no sea dígito
    val = val.replace(/[^0-9]/g, "");

    if (val === "") {
      onChange(NaN); // input vacío
      return;
    }

    const num = parseInt(val, 10);
    if (!isNaN(num)) {
      onChange(num);
    }
  };

  const handleBlur = () => {
    if (isNaN(value)) onChange(0); // si queda vacío → se pone en 0
  };

  const numericVal = parseInt(value.toString(), 10);
  const isOutOfRange =
    !isNaN(numericVal) && (numericVal < valMin || numericVal > valMax);

  return (
    <NoSpinnerInput
      size="small"
      variant={variant}
      type="number"
      label={label}
      value={isNaN(value) ? "" : value.toString()}
      onChange={handleChange}
      onBlur={handleBlur}
      inputProps={{
        inputMode: "numeric", // teclado numérico en móviles
        pattern: "[0-9]*",    // solo números positivos
        min: valMin,          // hint semántico, no reemplaza la validación
        max: valMax,
      }}
      error={isOutOfRange}
      sx={{ ...(sx || {}) }}
    />
  );
};

export default InputPositive;
