import React from "react";
import { styled } from "@mui/material/styles";
import CompactsInput from "./CompactsInput";

const NoSpinnerInput = styled(CompactsInput)(({ theme }) => ({
  "& input::-webkit-outer-spin-button, & input::-webkit-inner-spin-button": {
    WebkitAppearance: "none",
    margin: 0,
  },
  "& input[type=number]": {
    MozAppearance: "textfield", // Firefox
  },
}));

type CoordInputV2Props = {
  label: string;
  valMin: number;
  valMax: number;
  value: number;
  onChange: (val: number) => void;
  disabled?: boolean;
  onBlurCustom?: () => void;
};

const CoordInputV2: React.FC<CoordInputV2Props> = ({
  label,
  valMin,
  valMax,
  value,
  onChange,
  disabled = false,
  onBlurCustom,
}) => {
  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    
    let val = e.target.value;

    // Eliminar cualquier caracter que no sea dígito o el signo negativo al principio
    val = val.replace(/[^0-9-]/g, ""); // elimina puntos, comas, letras, etc.

    // Asegurar que solo haya un '-' y esté al principio
    if (val.includes("-")) {
      val = val.replace(/(?!^)-/g, "");
    }

    if (val === "" || val === "-") {
      onChange(NaN); // usamos NaN para indicar input "vacío" o solo "-"
      return;
    }

    const num = parseInt(val, 10);
    if (!isNaN(num)) {
      onChange(num);
    }
  };

  const handleBlur = () => {
    if (isNaN(value)) onChange(0); // si queda vacío o inválido → se pone en 0
    if (onBlurCustom) onBlurCustom();
  };

  const numericVal = parseInt(value.toString(), 10);
  const isOutOfRange =
    !isNaN(numericVal) && (numericVal < valMin || numericVal > valMax);

  return (
    <NoSpinnerInput
      type="number"
      label={label}
      value={isNaN(value) ? "" : value.toString()}
      onChange={handleChange}
      onBlur={handleBlur}
      disabled={disabled}
      inputProps={{
        inputMode: "numeric", // teclado numérico en móviles
        pattern: "-?[0-9]*", // permite negativo
      }}
      error={isOutOfRange}
    />
  );
};

export default CoordInputV2;
